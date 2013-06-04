#ifndef UTIL_H
#define UTIL_H

template <typename Iter>
typename Iter::value_type mean(typename Iter::value_type val,
                               Iter begin,
                               Iter end) {
    for (Iter i = begin; i != end; i++) {
        val += *i;
    }

    return val / (end - begin);
}

#endif
