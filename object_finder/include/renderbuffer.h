#ifndef RENDERBUFFER_H
#define RENDERBUFFER_H

#include <boost/foreach.hpp>

#include "image.h"

template<class T>
T clamp(T x, T lo, T hi) {
    return std::min(std::max(x, lo), hi);
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
    inline bool includes(int x) const {
        return x_start <= x && x < x_end;
    }
};

struct cmpclass {
    const std::vector<Segment> &segments;
    cmpclass(const std::vector<Segment> &segments) : segments(segments) { }
    bool operator() (unsigned int i,unsigned int j) {
        return segments[i].x_start < segments[j].x_start;
    }
};
struct ScanLine {
    std::vector<Segment> segments;
    
    void add_segment(const Segment &add) { // underwrites
        unsigned int size = segments.size();
        if(size) {
            Segment &lastseg = segments[size-1];
            if(lastseg.region == add.region) {
                if(lastseg.x_end == add.x_start) {
                    lastseg.x_end = add.x_end;
                    return;
                } else if(lastseg.x_start == add.x_end) {
                    lastseg.x_start = add.x_start;
                    return;
                }
            }
        }
        segments.push_back(add);
    }
    void finalize_layer() {
        BOOST_FOREACH(Segment &segment, segments) {
            segment.z_0 = std::numeric_limits<double>::infinity();
            segment.z_slope = 0;
        }
    }
    void accumulate_results(std::vector<Result> &results, const TaggedImage &img, unsigned int Y, std::vector<int> *dbg_image=NULL) const {
        unsigned int segs = segments.size();
        if(!segs)
            return;
        
        unsigned int indices[segs];
        for(unsigned int i = 0; i < segs; i++) {
            indices[i] = i;
        }
        std::sort(indices, indices + segs, cmpclass(segments));
        
        unsigned int best = indices[0];
        int start_x = segments[best].x_start;
        unsigned int seg_pos = 1;
        while(true) {
            int possible_end_x1 = best != segs ? segments[best].x_end : INT_MAX;
            int possible_end_x2 = seg_pos != segs ? segments[indices[seg_pos]].x_start : INT_MAX;
            
            int end_x = std::min(possible_end_x1, possible_end_x2);
            if(end_x == INT_MAX)
                break;
            
            if(best != segs && end_x != start_x) {
                const Segment &segment = segments[best];
                results[segment.region] += img.get_line_sum(Y, start_x, end_x);
                if(dbg_image) {
                    for(int X = start_x; X < end_x; X++) {
                        (*dbg_image)[Y * img.width + X] = segment.region + 10;
                    }
                }
            }
            
            if(end_x == possible_end_x2) { // a new segment overlapped this one
                best = std::min(best, indices[seg_pos]); // become it if it has a higher weight
                seg_pos++;
            } else { // this segment ended first
                while(best != segs && !segments[best].includes(end_x)) { // find the segment to fall down to
                    best++;
                }
            }
            
            start_x = end_x;
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
        scanlines.resize(img.height);
        BOOST_FOREACH(ScanLine &scanline, scanlines) {
            scanline.reset();
        }
        areas.clear();
    }
    void reset(const TaggedImage &img, const RenderBuffer &orig) {
        this->img = &img;
        
        assert(orig.scanlines.size() == img.height);
        scanlines.resize(img.height);
        
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
        for(unsigned int Y = 0; Y < img->height; Y++) {
            scanlines[Y].accumulate_results(results, *img, Y);
        }
        
        std::vector<ResultWithArea> results2;
        for(unsigned int i = 0; i < areas.size(); i++) {
            results2.push_back(ResultWithArea(results[i], areas[i]));
        }
        return results2;
    }
    std::vector<Result> draw_debug_regions(std::vector<int> &dbg_image) {
        std::vector<Result> results(areas.size(), Result::Zero());
        for(unsigned int Y = 0; Y < img->height; Y++) {
            scanlines[Y].accumulate_results(results, *img, Y, &dbg_image);
        }
        return results;
    }
};

#endif
