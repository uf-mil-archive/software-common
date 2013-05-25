#ifndef RENDERBUFFER_H
#define RENDERBUFFER_H

#include <boost/foreach.hpp>

#include "image.h"

struct Segment {
    int x_start, x_end; // [x_start, x_end)
    double z_0, z_slope; // z = z_0 + z_slope * x
    int region;
    Segment(int x_start, int x_end, double z_0, double z_slope, int region) :
        x_start(x_start), x_end(x_end), z_0(z_0), z_slope(z_slope), region(region) { }
    Segment clip(int x_start, int x_end) const {
        return Segment(x_start, x_end, z_0, z_slope, region);
    }
    double intersection_x(const Segment &other) const {
        return (other.z_0 - z_0)/(z_slope - other.z_slope);
    }
    bool overlaps(const Segment &other) const {
        int intersection_start = std::max(x_start, other.x_start), intersection_end = std::min(x_end, other.x_end);
        return intersection_start < intersection_end;
    }
    bool is_real() const {
        return x_start < x_end;
    }
    double z(double x) const {
        return z_0 + z_slope * x;
    }
};

struct ScanLine {
    std::vector<Segment> segments;
    void add_segment(const Segment &add) {
        // currently this ignores depth and new areas always overwrite old ones
        std::vector<Segment> new_segments;
        BOOST_FOREACH(const Segment &segment, segments) {
            Segment new_segment = segment.clip(std::min(segment.x_start, add.x_start), std::min(segment.x_end, add.x_start));
            if(new_segment.is_real()) {
                new_segments.push_back(new_segment);
            }
        }
        new_segments.push_back(add);
        BOOST_FOREACH(const Segment &segment, segments) {
            Segment new_segment = segment.clip(std::max(segment.x_start, add.x_end), std::max(segment.x_end, add.x_end));
            if(new_segment.is_real()) {
                new_segments.push_back(new_segment);
            }
        }
        segments = new_segments;
        /*
        std::vector<Segment> new_segments;
        BOOST_FOREACH(const Segment &segment, segments) {
            if(not add.overlaps(segment)) {
                new_segments.push_back(segment);
                continue;
            }
            double x = add.intersection_x(segment);
            std::vector<int> xs(5);
            xs.push_back(add.x_start);
            xs.push_back(segment.x_start);
            xs.push_back(add.x_end);
            xs.push_back(segment.x_end);
            if(std::min(add.x_start, segment.x_start) <= x and x < std::max(add.x_end, segment.x_end))
                xs.push_back((int)(x + .5));
            sort(xs.begin(), xs.end());
            for(unsigned int i = 0; i < xs.size()-1; i++) {
                int x1 = xs[i];
                int x2 = xs[i+1];
                double midx = .5*(x1+x2);
                if(add.z(midx) > segment.z(midx)) {
                    new_segments.push_back(add.clip(x1, x2));
                } else {
                    new_segments.push_back(segment.clip(x1, x2));
                }
            }
        }
        segments = new_segments;
        */
    }
    void finalize_layer() {
        BOOST_FOREACH(Segment &segment, segments) {
            segment.z_0 = std::numeric_limits<double>::infinity();
            segment.z_slope = 0;
        }
    }
    void accumulate_results(std::vector<Result> &results, const TaggedImage &img, unsigned int Y) {
        BOOST_FOREACH(const Segment &segment, segments) {
            results[segment.region] += img.get_line_sum(Y, segment.x_start, segment.x_end);
        }
    }
};

struct RenderBuffer {
    typedef int RegionType;
    
    const TaggedImage &img;
    std::vector<ScanLine> scanlines;
    std::vector<double> areas;
    RenderBuffer(const TaggedImage &img) : img(img) {
        scanlines.resize(img.cam_info.height);
    }
    RegionType new_region() {
        areas.push_back(0);
        return areas.size()-1;
    }
    void finalize_layer() {
        BOOST_FOREACH(ScanLine &scanline, scanlines) {
            scanline.finalize_layer();
        }
    }
    std::vector<ResultWithArea> get_results() {
        std::vector<Result> results(areas.size(), Result::Zero());
        for(unsigned int Y = 0; Y < img.cam_info.height; Y++) {
            scanlines[Y].accumulate_results(results, img, Y);
        }
        
        std::vector<ResultWithArea> results2;
        for(unsigned int i = 0; i < areas.size(); i++) {
            results2.push_back(ResultWithArea(results[i], areas[i]));
        }
        return results2;
    }
};

#endif
