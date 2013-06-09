#ifndef RENDERBUFFER_H
#define RENDERBUFFER_H

#include <queue>

#include <boost/foreach.hpp>

#include "image.h"

template<class T>
T clamp(T x, T lo, T hi) {
    return min(max(x, lo), hi);
}

struct Segment {
    int x_start, x_end; // [x_start, x_end)
    double z_0, z_slope; // z = z_0 + z_slope * x
    int region;
    Segment() { }
    Segment(int x_start, int x_end, double z_0, double z_slope, int region) :
        x_start(x_start), x_end(x_end), z_0(z_0), z_slope(z_slope), region(region) { }
    Segment set_bounds(int x_start, int x_end) const {
        return Segment(x_start, x_end, z_0, z_slope, region);
    }
    Segment clip(int x_start, int x_end) const {
        return Segment(
            clamp(this->x_start, x_start, x_end),
            clamp(this->x_end  , x_start, x_end),
        z_0, z_slope, region);
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

struct cmpclass {
    const std::vector<Segment> &segments;
    cmpclass(const std::vector<Segment> &segments) : segments(segments) { }
    bool operator() (int i,int j) {
        int x1 = i >= 0 ? segments[i].x_start : segments[-i-1].x_end;
        int x2 = j >= 0 ? segments[j].x_start : segments[-j-1].x_end;
        if(x1 == x2) {
            return i > j; // always have start before end
        }
        return x1 < x2;
    }
};
struct ScanLine {
    std::vector<Segment> segments;
    
    void add_segment(const Segment &add) { // underwrites
        segments.push_back(add);
    }
    void finalize_layer() {
        BOOST_FOREACH(Segment &segment, segments) {
            segment.z_0 = std::numeric_limits<double>::infinity();
            segment.z_slope = 0;
        }
    }
    void accumulate_results(std::vector<Result> &results, const TaggedImage &img, unsigned int Y, std::vector<int> *dbg_image=NULL) {
        unsigned int indices_size = segments.size()*2;
        int indices[indices_size];
        for(unsigned int i = 0; i < segments.size(); i++) {
            indices[i] = i;
        }
        for(unsigned int i = 0; i < segments.size(); i++) {
            indices[segments.size()+i] = -i-1;
        }
        std::sort(indices, indices + indices_size, cmpclass(segments));
        
        bool active[segments.size()];
        for(unsigned int i = 0; i < segments.size(); i++) {
            active[i] = false;
        }
        static std::priority_queue<int> activelist;
        assert(activelist.empty());
        int last_x = -1;
        for(unsigned int i = 0; i < indices_size; i++) {
            //std::cout << "INDEX " << i << std::endl;
            // find current top segment
            int best = -1;
            if(last_x >= 0) {
                while(!activelist.empty()) {
                    if(active[-activelist.top()]) {
                        best = -activelist.top();
                        break;
                    } else {
                        activelist.pop();
                    }
                }
            }
            /*
            int best2 = -1;
            for(unsigned int j = 0; j < segments.size(); j++) {
                if(active[j]) {
                    best2 = j;
                    break;
                }
            }
            assert(best == best2);
            */
            //std::cout << "  BEST " << best << std::endl;
            
            // apply start or end
            int index = indices[i];
            int this_x;
            if(index >= 0) { // start
                int seg = index;
                //assert(!active[seg]);
                active[seg] = true;
                activelist.push(-seg);
                this_x = segments[seg].x_start;
                //std::cout << "START " << seg << std::endl;
            } else { // end
                int seg = -index-1;
                //assert(active[seg]);
                active[seg] = false;
                this_x = segments[seg].x_end;
                //std::cout << "END " << seg << std::endl;
            }
            
            if(last_x >= 0) {
                //assert(this_x >= last_x);
                if(best != -1 && this_x > last_x) {
                    const Segment &segment = segments[best];
                    //assert(*last_x >= segment.x_start);
                    //assert(this_x <= segment.x_end);
                    results[segment.region] += img.get_line_sum(Y, last_x, this_x);
                    if(dbg_image) {
                        for(int X = last_x; X < this_x; X++) {
                            (*dbg_image)[Y * img.cam_info.width + X] = segment.region + 10;
                        }
                    }
                }
            }
            
            last_x = this_x;
        }
        while(!activelist.empty()) {
            activelist.pop();
        }
    }
    void reset() {
        segments.clear();
    }
    void reset(const ScanLine &other) {
        segments.resize(other.segments.size());
        for(unsigned int i = 0; i < other.segments.size(); i++) {
            segments[i] = other.segments[i];
        }
    }
};

struct RenderBuffer {
    typedef int RegionType;
    
    const TaggedImage *img; // pointer since we want to reassign it
    std::vector<ScanLine> scanlines;
    std::vector<double> areas;
    RenderBuffer() { img = NULL; }
    RenderBuffer(const TaggedImage &img) { reset(img); }
    void reset(const TaggedImage &img) {
        this->img = &img;
        scanlines.resize(img.cam_info.height);
        BOOST_FOREACH(ScanLine &scanline, scanlines) {
            scanline.reset();
        }
        areas.clear();
    }
    void reset(const TaggedImage &img, const RenderBuffer &orig) {
        this->img = &img;
        
        assert(orig.scanlines.size() == img.cam_info.height);
        scanlines.resize(img.cam_info.height);
        
        int i = 0; BOOST_FOREACH(ScanLine &scanline, scanlines) {
            scanline.reset(orig.scanlines[i]);
         i++; }
        areas = orig.areas;
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
        for(unsigned int Y = 0; Y < img->cam_info.height; Y++) {
            scanlines[Y].accumulate_results(results, *img, Y);
        }
        
        std::vector<ResultWithArea> results2;
        for(unsigned int i = 0; i < areas.size(); i++) {
            results2.push_back(ResultWithArea(results[i], areas[i]));
        }
        return results2;
    }
    void draw_debug_regions(std::vector<int> &dbg_image) {
        std::vector<Result> results(areas.size(), Result::Zero());
        for(unsigned int Y = 0; Y < img->cam_info.height; Y++) {
            scanlines[Y].accumulate_results(results, *img, Y, &dbg_image);
        }
    }
};

#endif
