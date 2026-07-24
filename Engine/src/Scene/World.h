#pragma once
#include "Entity.h"
#include "SparseSet.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <typeindex>
#include <cstdint>

namespace SGE {

// The World owns all entities and component pools.
//
// Entities are integer handles, not objects: creating one hands back an
// Entity; components are stored in per-type sparse sets keyed by std::type_index.
// This is the data-oriented layout that doubles as your DOD showcase — all
// components of a type live contiguously, so systems iterate them linearly.
class World {
public:
    Entity Create() {
        uint32_t index;
        if (!m_freeIndices.empty()) {
            index = m_freeIndices.back();
            m_freeIndices.pop_back();
        } else {
            index = static_cast<uint32_t>(m_generations.size());
            m_generations.push_back(0);
        }
        return Entity::Make(index, m_generations[index]);
    }

    void Destroy(Entity e) {
        if (!IsAlive(e)) return;
        for (auto& [type, pool] : m_pools)
            pool->Remove(e);
        // Bump the generation so any surviving handles to this slot go stale.
        m_generations[e.Index()]++;
        m_freeIndices.push_back(e.Index());
    }

    bool IsAlive(Entity e) const {
        const uint32_t idx = e.Index();
        return idx < m_generations.size()
            && m_generations[idx] == e.Generation();
    }

    template <typename T>
    T& Add(Entity e, const T& value = {}) { return Pool<T>().Add(e, value); }

    template <typename T>
    void Remove(Entity e) { if (auto* p = TryPool<T>()) p->Remove(e); }

    template <typename T>
    bool Has(Entity e) { auto* p = TryPool<T>(); return p && p->Has(e); }

    template <typename T>
    T& Get(Entity e) { return Pool<T>().Get(e); }

    template <typename T>
    T* TryGet(Entity e) { auto* p = TryPool<T>(); return p ? p->TryGet(e) : nullptr; }

    // Iterate every entity that has *all* of the listed components, calling
    // fn(Entity, T&, Rest&...). The FIRST listed type drives iteration, so for
    // best performance list the rarest component first (e.g. View<MeshComponent,
    // TransformComponent>, since fewer things are renderable than transformed).
    //
    // Safe to Add components from inside fn; do NOT Destroy entities or remove
    // the driving component type mid-iteration (it would invalidate the loop).
    template <typename T, typename... Rest, typename Fn>
    void View(Fn&& fn) {
        SparseSet<T>* driver = TryPool<T>();
        if (!driver) return;
        // Walk backwards so an Add inside fn (which may push onto the driving
        // pool) can't shift elements we haven't visited yet.
        for (std::size_t i = driver->Size(); i-- > 0; ) {
            Entity e = driver->EntityAt(i);
            if constexpr (sizeof...(Rest) == 0) {
                fn(e, driver->ComponentAt(i));
            } else {
                if ((Has<Rest>(e) && ...))
                    fn(e, driver->ComponentAt(i), Get<Rest>(e)...);
            }
        }
    }

private:
    template <typename T>
    SparseSet<T>& Pool() {
        const std::type_index key(typeid(T));
        auto it = m_pools.find(key);
        if (it == m_pools.end())
            it = m_pools.emplace(key, std::make_unique<SparseSet<T>>()).first;
        return *static_cast<SparseSet<T>*>(it->second.get());
    }

    template <typename T>
    SparseSet<T>* TryPool() {
        const std::type_index key(typeid(T));
        auto it = m_pools.find(key);
        return it == m_pools.end() ? nullptr
             : static_cast<SparseSet<T>*>(it->second.get());
    }

    std::vector<uint32_t> m_generations; // slot index -> current generation
    std::vector<uint32_t> m_freeIndices; // recycled slot indices
    std::unordered_map<std::type_index, std::unique_ptr<ISparseSet>> m_pools;
};

} // namespace SGE
