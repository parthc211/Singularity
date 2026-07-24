#pragma once
#include "Entity.h"
#include <vector>
#include <cstdint>
#include <cassert>
#include <utility>

namespace SGE {

// Type-erased interface so World can hold a map of heterogeneous component
// pools and remove a destroyed entity from all of them without knowing T.
class ISparseSet {
public:
    virtual ~ISparseSet() = default;
    virtual void Remove(Entity e) = 0;
    virtual bool Has(Entity e) const = 0;
};

// A sparse set maps entity indices to a tightly-packed (dense) array of T:
//
//   m_sparse[entityIndex] -> position in the dense arrays
//   m_dense[i]            -> the Entity that owns m_components[i]
//   m_components[i]       -> the component data (parallel to m_dense)
//
// Iterating m_components is cache-friendly; Add/Get/Remove are O(1). Remove
// uses swap-and-pop, so component order is not stable across removals.
template <typename T>
class SparseSet : public ISparseSet {
public:
    static constexpr uint32_t Tombstone = 0xFFFFFFFFu;

    bool Has(Entity e) const override {
        const uint32_t idx = e.Index();
        return idx < m_sparse.size()
            && m_sparse[idx] != Tombstone
            && m_dense[m_sparse[idx]] == e; // also rejects stale generations
    }

    T& Add(Entity e, const T& value) {
        const uint32_t idx = e.Index();
        if (idx >= m_sparse.size())
            m_sparse.resize(idx + 1, Tombstone);

        // Already present (same entity): overwrite in place.
        if (m_sparse[idx] != Tombstone && m_dense[m_sparse[idx]] == e) {
            m_components[m_sparse[idx]] = value;
            return m_components[m_sparse[idx]];
        }

        m_sparse[idx] = static_cast<uint32_t>(m_dense.size());
        m_dense.push_back(e);
        m_components.push_back(value);
        return m_components.back();
    }

    void Remove(Entity e) override {
        const uint32_t idx = e.Index();
        if (idx >= m_sparse.size() || m_sparse[idx] == Tombstone) return;
        if (m_dense[m_sparse[idx]] != e) return; // stale handle, ignore

        const uint32_t denseIdx = m_sparse[idx];
        const uint32_t lastIdx  = static_cast<uint32_t>(m_dense.size() - 1);

        // Move the last element into the hole, then shrink.
        m_dense[denseIdx]      = m_dense[lastIdx];
        m_components[denseIdx] = std::move(m_components[lastIdx]);
        m_sparse[m_dense[denseIdx].Index()] = denseIdx;

        m_dense.pop_back();
        m_components.pop_back();
        m_sparse[idx] = Tombstone;
    }

    T* TryGet(Entity e) {
        const uint32_t idx = e.Index();
        if (idx >= m_sparse.size() || m_sparse[idx] == Tombstone) return nullptr;
        if (m_dense[m_sparse[idx]] != e) return nullptr;
        return &m_components[m_sparse[idx]];
    }

    T& Get(Entity e) {
        T* p = TryGet(e);
        assert(p && "SparseSet::Get on an entity without this component");
        return *p;
    }

    // Dense iteration helpers (used by World::View).
    std::size_t Size()                    const { return m_dense.size(); }
    Entity      EntityAt(std::size_t i)   const { return m_dense[i]; }
    T&          ComponentAt(std::size_t i)      { return m_components[i]; }

private:
    std::vector<uint32_t> m_sparse;     // entity index -> dense index
    std::vector<Entity>   m_dense;      // dense index  -> entity
    std::vector<T>        m_components; // dense index  -> component
};

} // namespace SGE
