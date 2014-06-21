#include <boost/foreach.hpp>

#include "Contours.h"
#include "Normalizer.h"
#include "Thresholder.h"
#include "Blob.h"

#include "BinsFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

template<typename T>
vector<vector<T> > cartesian_product(vector<vector<T> > & x) {
    vector<vector<T> > result;
    vector<int> indices(x.size(), 0);
    while(true) {
        for(int i = 0; i < x.size(); i++) {
            indices[i]++;
            if(indices[i] == x[i].size()) {
                indices[i] = 0;
            } else {
                break;
            }
            if(i == x.size() - 1) {
                return result;
            }
        }
        result.push_back(vector<T>());
        for(int i = 0; i < x.size(); i++) {
            result[result.size()-1].push_back(x[i][indices[i]]);
        }
    }
}

optional<Mat> extract_bottom(const Mat bin) {
    Mat normalized = Normalizer::normRGB(bin);
    vector<Mat> normBGR; split(normalized, normBGR);
    Mat bottom; threshold(normBGR[0], bottom, 0, 255, THRESH_OTSU|THRESH_BINARY_INV);
	Blob blobs(bottom, 0, 1e12, 1e12);
	if(!blobs.data.size()) return none;
	Blob::BlobData &blob = blobs.data[0];
	Size bottom_size(300, 150);
	Point2f bottom_src[4];
	Point2f perp_dir(1/blob.radius*-blob.direction.y, 1/blob.radius*blob.direction.x);
	for(int i = 0; i < 4; i++) {
	    double a = i==1||i==2 ? 0.5 : -0.5;
	    double b = i==2||i==3 ? 0.5 : -0.5;
	    bottom_src[i] = blob.rect_center + a*1/blob.radius*blob.direction*blob.long_length + b*perp_dir*blob.short_length;
    }
    Point2f bottom_dst[4];
    bottom_dst[0] = Point2f(0, 0);
    bottom_dst[1] = Point2f(bottom_size.width, 0);
    bottom_dst[2] = Point2f(bottom_size.width, bottom_size.height);
    bottom_dst[3] = Point2f(0, bottom_size.height);
    Mat bottom_t = getPerspectiveTransform(bottom_src, bottom_dst);
    Mat bottom2;warpPerspective(bin, bottom2, bottom_t, bottom_size);	
    return bottom2;
}

IFinder::FinderResult BinsFinder::find(const subjugator::ImageSource::Image &img) {
    Mat blurred; GaussianBlur(img.image, blurred, Size(0,0), 1.3);

    Thresholder thresholder(blurred);

    vector<property_tree::ptree> resultVector;
    Mat dbg = thresholder.black();

    Contours contours(dbg, config.get<float>("minContour"), config.get<float>("maxContour"), config.get<float>("maxPerimeter"));

    // Draw result
    Mat res = img.image.clone();
    contours.drawResult(res, CV_RGB(255, 255, 255));

    if(objectPath[0] == "all") {
        if(contours.boxes.size() && !(contours.boxes.size() == 1 && contours.boxes[0].touches_edge)) {
            Point centroidOfBoxes = contours.calcCentroidOfAllBoxes();
            circle(res,centroidOfBoxes, 5, CV_RGB(255,140,0), -1,8);
            property_tree::ptree fResult;
            fResult.put_child("center", Point_to_ptree(centroidOfBoxes, img));
            fResult.put("number_of_boxes", contours.boxes.size());
            // Scale returns the number of boxes that are currently being found.
            // The idea is to align to centroid until 4 boxes are found.
            fResult.put("angle", contours.calcAngleOfAllBoxes());
            fResult.put_child("direction", Direction_to_ptree(centroidOfBoxes, contours.calcDirectionOfAllBoxes(), img));
            fResult.put("direction_symmetry", 2);
            resultVector.push_back(fResult);
        }
    } else { assert(objectPath[0] == "single" || objectPath[0] == "single_save");
        vector<Mat> bins;
        vector<Point> centroids;
        BOOST_FOREACH(const Contours::OuterBox &box, contours.boxes) {
            bool touches_edge = false;
            BOOST_FOREACH(const Point& p, box.corners)
                if(p.x <= 1 || p.x >= img.image.cols-2 || p.y <= 1 || p.y >= img.image.rows-2)
                    touches_edge = true;
            if(touches_edge)
                    continue;

            Point2f src[4];
            for(unsigned int n = 0; n < box.corners.size(); n++)
                src[n] = Point2f(box.corners[n].x, box.corners[n].y);
            if(!(norm(src[1] - src[0]) > norm(src[3] - src[0]))) // make sure long edge matches long edge so image isn't squished
                for(unsigned int n = 0; n < box.corners.size(); n++)
                    src[n] = Point2f(box.corners[(n+1)%4].x, box.corners[(n+1)%4].y);

            Size bin_size(300, 150);
            
            int crop = 5;
            Point2f dst[4];
            dst[0] = Point2f(-2*crop, -crop);
            dst[1] = Point2f(bin_size.width+2*crop, -crop);
            dst[2] = Point2f(bin_size.width+2*crop, bin_size.height+crop);
            dst[3] = Point2f(-2*crop, bin_size.height+crop);

            Mat t = getPerspectiveTransform(src, dst);
            Mat bin;warpPerspective(img.image, bin, t, bin_size);
            
            Mat normalized = Normalizer::normRGB(bin);
            warpPerspective(normalized, res, t, img.image.size(), WARP_INVERSE_MAP, BORDER_TRANSPARENT);
            
            optional<Mat> bottom = bin; //extract_bottom(bin);
            if(!bottom) continue;
            bins.push_back(*bottom);
            centroids.push_back(box.centroid);
            
            
            property_tree::ptree fResult;
            fResult.put_child("center", Point_to_ptree(box.centroid, img));
            fResult.put_child("direction", Direction_to_ptree(box.centroid, src[1] - src[0], img));
            //fResult.put("direction_symmetry", 2);

            fResult.put("angle", box.angle);
            fResult.put("scale", box.area);
            resultVector.push_back(fResult);
        }
        
        if(objectPath[0] == "single_save" && resultVector.size() == names.size() ) {
            for(int bin_index = 0; bin_index < names.size(); bin_index++) {
                ostringstream ss;
                ss << config.get<string>("imagePath") << "/sample" << bin_index << ".png";
                imwrite(ss.str(), bins[bin_index]);
            }
        } else if(objectPath[0] == "single" && resultVector.size() == names.size()) {
            vector<vector<vector<vector<double> > > > distances; // template_index, template_index2, flipped, bin_index
            for(int template_index = 0; template_index < templates.size(); template_index++) {
            distances.push_back(vector<vector<vector<double> > >());
            for(int template_index2 = 0; template_index2 < templates[template_index].size(); template_index2++) {
                distances[template_index].push_back(vector<vector<double> >());
                Mat b = Normalizer::normRGB(templates[template_index][template_index2]);
                for(int flipped = 0; flipped < 2; flipped++) {
                    distances[template_index][template_index2].push_back(vector<double>());
                    if(flipped) {
                        flip(b, b, -1);
                    }
                    for(int bin_index = 0; bin_index < names.size(); bin_index++) {
                        Mat a = Normalizer::normRGB(bins[bin_index]);
                        
                        double distance = 0;
                        assert(a.rows == b.rows && a.cols == b.cols);
                        for(int i = 0; i < a.rows; i++) {
                            for(int j = 0; j < a.cols; j++) {
                                Vec3b a_pixel = a.at<Vec3b>(i, j);
                                Vec3b b_pixel = b.at<Vec3b>(i, j);
                                distance += abs((double)a_pixel[0] - b_pixel[0]) +
                                            abs((double)a_pixel[1] - b_pixel[1]) +
                                            abs((double)a_pixel[2] - b_pixel[2]);
                            }
                        }
                        
                        distances[template_index][template_index2][flipped].push_back(distance);
                    }
                    if(flipped) {
                        flip(b, b, -1);
                    }
                }
            }
            }
            
            typedef pair<int, int> Index;
            
            vector<vector<Index> > indices_all;
            int i = 0; BOOST_FOREACH(const vector<string> & subnames, names) {
                indices_all.push_back(vector<Index>());
                for(int j = 0; j < subnames.size(); j++) {
                    indices_all[indices_all.size()-1].push_back(make_pair(i, j));
                }
            i++; }
            
            std::vector<Index> best_indices;
            optional<double> best_score;
            BOOST_FOREACH(std::vector<Index> indices, cartesian_product(indices_all)) {
                do {
                    for(int rotate_mask = 0; rotate_mask < 16; rotate_mask++) {
                        double dist = 0;
                        for(int bin_index = 0; bin_index < names.size(); bin_index++) {
                            dist += distances[indices[bin_index].first][indices[bin_index].second][rotate_mask & (1<<bin_index) ? 1 : 0][bin_index];
                        }
                        if(!best_score || dist < *best_score) {
                            best_score = dist;
                            best_indices = indices;
                        }
                    }
                } while(next_permutation(indices.begin(), indices.end()));
            }
            for(int i = 0; i < names.size(); i++) {
                resultVector[i].put("image_text", names[best_indices[i].first][best_indices[i].second]);
                ostringstream ss; ss << names[best_indices[i].first][best_indices[i].second];
                putText(res,ss.str().c_str(),centroids[i],FONT_HERSHEY_SIMPLEX,1,CV_RGB(0,0,255),3);
            }
        }
    }
    return FinderResult(resultVector, res, dbg);
}
