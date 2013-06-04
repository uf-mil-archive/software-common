#ifndef RENDERBUFFER_H
#define RENDERBUFFER_H

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


    
static void append(Segment *list, unsigned int &size, const Segment &new_segment) {
    if(size && list[size-1].x_end == new_segment.x_start && list[size-1].region == new_segment.region) {
        list[size-1].x_end = new_segment.x_end;
    } else {
        list[size] = new_segment; size++;
    }
}

struct ScanLine {
    std::vector<Segment> segments;
    
    void add_segment(const Segment &add) { // underwrites
        std::vector<Segment> old_segments; old_segments.swap(segments);
        
        segments.push_back(add);
        BOOST_FOREACH(const Segment &segment, old_segments) {
            overwrite_segment(segment);
        }
    }
    void overwrite_segment(const Segment &add) {
        Segment new_segments[2*segments.size() + 1];
        unsigned int new_segments_size = 0;
        
        // currently this ignores depth and new areas always overwrite old ones
        BOOST_FOREACH(const Segment &segment, segments) {
            Segment new_segment = segment.set_bounds(std::min(segment.x_start, add.x_start), std::min(segment.x_end, add.x_start));
            if(new_segment.is_real()) {
                new_segments[new_segments_size] = new_segment; new_segments_size++;
            }
        }
        append(new_segments, new_segments_size, add);
        BOOST_FOREACH(const Segment &segment, segments) {
            Segment new_segment = segment.set_bounds(std::max(segment.x_start, add.x_end), std::max(segment.x_end, add.x_end));
            if(new_segment.is_real()) {
                append(new_segments, new_segments_size, new_segment);
            }
        }
        
        segments.resize(new_segments_size);
        for(unsigned int i = 0; i < new_segments_size; i++) {
            segments[i] = new_segments[i];
        }
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
    void reset() {
        segments.clear();
    }
};

struct RenderBuffer {
    typedef int RegionType;
    
    const TaggedImage *img; // pointer since we want to reassign it
    std::vector<ScanLine> scanlines;
    std::vector<double> areas;
    RenderBuffer(const TaggedImage &img) { reset(img); }
    void reset(const TaggedImage &img) {
        this->img = &img;
        scanlines.resize(img.cam_info.height);
        BOOST_FOREACH(ScanLine &scanline, scanlines) {
            scanline.reset();
        }
        areas.clear();
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
        for(unsigned int Y = 0; Y < img->cam_info.height; Y++) {
            ScanLine &scanline = scanlines[Y];
            BOOST_FOREACH(const Segment &segment, scanline.segments) {
                for(int X = segment.x_start; X < segment.x_end; X++) {
                    dbg_image[Y * img->cam_info.width + X] = segment.region + 1;
                }
            }
        }
    }
};

#endif
