#ifndef MULTIARRY_H
#define MULTIARRY_H

#include <cstddef>
#include <vector>

template <typename T>
class MultiArray
{
public:
    explicit MultiArray(const std::vector<size_t>& dimensions) :
        dimensions(dimensions),
        values(computeTotalSize(dimensions))
    {
        assert(!dimensions.empty());
        assert(!values.empty());
    }

    const T& get(const std::vector<size_t>& indexes) const
    {
        return values[computeIndex(indexes)];
    }
    T& get(const std::vector<size_t>& indexes)
    {
        return values[computeIndex(indexes)];
    }

    size_t computeIndex(const std::vector<size_t>& indexes) const
    {
        assert(indexes.size() == dimensions.size());

        size_t index = 0;
        size_t mul = 1;

        for (size_t i = 0; i != dimensions.size(); ++i) {
            assert(indexes[i] < dimensions[i]);
            index += indexes[i] * mul;
            mul *= dimensions[i];
        }
        assert(index < values.size());
        return index;
    }

    std::vector<size_t> computeIndexes(size_t index) const
    {
        assert(index < values.size());

        std::vector<size_t> res(dimensions.size());

        size_t mul = values.size();
        for (size_t i = dimensions.size(); i != 0; --i) {
            mul /= dimensions[i - 1];
            res[i - 1] = index / mul;
            assert(res[i - 1] < dimensions[i - 1]);
            index -= res[i - 1] * mul;
        }
        return res;
    }

private:
    size_t computeTotalSize(const std::vector<size_t>& dimensions) const
    {
        size_t totalSize = 1;

        for (auto i : dimensions) {
            totalSize *= i;
        }
        return totalSize;
    }

private:
    std::vector<size_t> dimensions;
    std::vector<T> values;
};

#endif