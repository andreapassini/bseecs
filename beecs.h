#ifndef BSEECS_ECS_H
#define BSEECS_ECS_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <bitset>
#include <memory>
#include <type_traits>
#include <cassert>

// Can replace these defines with custom macros elsewhere
#ifndef BSEECS_ASSERTS
	#define BSEECS_ASSERT(condition, msg) \
		if (!(condition)) { \
			std::cerr << "[BSEECS error]: " << msg << std::endl; \
			::abort(); \
		}
#endif
#ifndef BSEECS_INFO
	#ifdef BSEECS_INFO_ENABLED
		#define BSEECS_INFO(msg) std::cout << "[BSEECS info]: " << msg << "\n";
	#else
		#define BSEECS_INFO(msg);
	#endif
#endif

namespace bseecs {

	// In ECS, entities are simply just indices which group data
	using EntityID = uint64_t;

	static constexpr EntityID NULL_ENTITY = std::numeric_limits<EntityID>::max();

	// Max amount of entities alive at once.
	// Set this to NULL_ENTITY if you want no limit.
	// Once limit is hit, an assert will fire and
	// the program will terminate.
	constexpr size_t MAX_ENTITIES = 1'000'000;

	// Should be a multiple of 32 (4 bytes), since
	// bitset overallocates by 4 bytes each time.
	constexpr size_t MAX_COMPONENTS = 64;

	// Base class allows runtime polymorphism
	class ISparseSet {
	public:
		virtual ~ISparseSet() = default;
		virtual void Delete(EntityID) = 0;
		virtual void Clear() = 0;
	};

	/*
	*  A templated sparse set implementation, mapping EntityID -> T
	* 
	*  - Get(EntityID): returns T or NULL if EntityID is not in sparse set
	*  - Set(EntityID, T&&): Adds/Overwrites into the dense list for the specified entity
	*  - Delete(EntityID): Removes data for EntityID from dense list
	*/
	template <typename T>
	class SparseSet: public ISparseSet {
	private:

		using Sparse = std::vector<size_t>;

		std::vector<Sparse> m_sparsePages;

		std::vector<T> m_dense;
		std::vector<EntityID> m_denseToEntity; // 1:1 vector where dense index == Entity Index

		const size_t SPARSE_MAX_SIZE = 1'000;

		static constexpr size_t tombstone = std::numeric_limits<size_t>::max();

		/*
		* Inserts a given dense index into the sparse vector, associating
		* an Entity ID with the index in the dense vector.
		*
		* This doesnt actually insert anything into the dense
		* vector, it simply defines a mapping from ID -> index
		*/
		void SetDenseIndex(EntityID id, size_t index) {
			size_t page = id / SPARSE_MAX_SIZE;
			size_t sparseIndex = id % SPARSE_MAX_SIZE; // Index local to a page

			if (page >= m_sparsePages.size())
				m_sparsePages.resize(page + 1);

			Sparse& sparse = m_sparsePages[page];
			if (sparseIndex >= sparse.size())
				sparse.resize(sparseIndex + 1, tombstone);

			sparse[sparseIndex] = index;
		}

		/*
		* Returns the dense index for a given entity ID,
		* or a tombstone (null) value if non-existent
		*/
		size_t GetDenseIndex(EntityID id) {
			size_t page = id / SPARSE_MAX_SIZE;
			size_t sparseIndex = id % SPARSE_MAX_SIZE;

			if (page < m_sparsePages.size()) {
				Sparse& sparse = m_sparsePages[page];
				if (sparseIndex < sparse.size())
					return sparse[sparseIndex];
			}

			return tombstone;
		}

	public:

		SparseSet() {
			// Avoids initial copies/allocation, feel free to alter size
			m_dense.reserve(100);
		}

		T* Set(EntityID id, T obj) {
			// If index already exists, then simply overwrite
			// that element in dense list, no need to delete
			size_t index = GetDenseIndex(id);
			if (index != tombstone) {
				m_dense[index] = obj;
				m_denseToEntity[index] = id;

				return &m_dense[index];
			}

			// New index will be the back of the dense list
			SetDenseIndex(id, m_dense.size());

			m_dense.push_back(obj);
			m_denseToEntity.push_back(id);

			return &m_dense.back();
		}

		T* Get(EntityID id) {
			size_t index = GetDenseIndex(id);
			return (index != tombstone) ? &m_dense[index] : nullptr;
		}

		T& GetRef(EntityID id)
		{
			size_t index = GetDenseIndex(id);
			return m_dense[index];
			//return (index != tombstone) ? m_dense[index] : nullptr;
		}


		EntityID GetEntity(EntityID compId)
		{
			return m_denseToEntity[compId];
		}
		
		void Delete(EntityID id) override {

			size_t deletedIndex = GetDenseIndex(id);
			BSEECS_ASSERT(deletedIndex != tombstone && !m_dense.empty(), "Trying to delete non-existent entity in sparse set");

			SetDenseIndex(m_denseToEntity.back(), deletedIndex);
			SetDenseIndex(id, tombstone);

			std::swap(m_dense.back(), m_dense[deletedIndex]);
			std::swap(m_denseToEntity.back(), m_denseToEntity[deletedIndex]);

			m_dense.pop_back();
			m_denseToEntity.pop_back();
		}

		void Clear() override {
			m_dense.clear();
			m_sparsePages.clear();
			m_denseToEntity.clear();
		}

		bool IsEmpty() const {
			return m_dense.empty();
		}

		// Dense list
		std::vector<T>& Data()
		{
			return m_dense;
		}

		void PrintDense() {
			std::stringstream ss;
			std::string delim = "";
			for (const T& e : m_dense) {
				ss << delim << e;
				if (delim.empty())
					delim = ", ";
			}
			BSEECS_INFO("[" << ss.str() << "]");
		}

	};


	class ECS {
	private:

		// Each bit in the mask represents a component,
		// '1' == active, '0' == inactive.
		using ComponentMask = std::bitset<MAX_COMPONENTS>;


		using TypeName = const char*;


		// List of IDs already created, but no longer in use
		std::vector<EntityID> m_availableEntities;


		// Associates ID with name provided in CreateEntity(), mainly for debugging
		std::unordered_map<EntityID, std::string> m_entityNames;


		// Holds generic pointers to specific component sparse sets.
		// 
		// Index into this array using the corresponding bit position
		// found by using m_componentBitPosition
		std::vector<std::unique_ptr<ISparseSet>> m_componentPools;

		struct ComponentInfo
		{
			size_t m_bitPosition{};
			ComponentMask m_requiredComponents{};
			ComponentMask m_isRequiredInComponents{};
		};

		// Key is component name, value is the bit position in ComponentMask
		std::unordered_map<TypeName, ComponentInfo> m_componentBitPosition;


		// Highest recorded entity ID
		EntityID m_maxEntityID = 0;


		static constexpr size_t tombstone = std::numeric_limits<size_t>::max();


		#define ENTITY_INFO(id) \
			"['" << GetEntityName(id) << "', ID: " << id << "]"

		#define BSEECS_ASSERT_VALID_ENTITY(id) \
			BSEECS_ASSERT(id != NULL_ENTITY, "NULL_ENTITY cannot be operated on by the ECS") \
			BSEECS_ASSERT(id < m_maxEntityID && id >= 0, "Invalid entity ID out of bounds: " << id);

	
	private:

		template <typename T>
		size_t GetComponentBitPosition() {
			TypeName name = typeid(T).name();
			auto it = m_componentBitPosition.find(name);
			if (it == m_componentBitPosition.end())
				return tombstone;

			return it->second.m_bitPosition;
		}

		template <typename T>
		ComponentMask* const GetRequiredComponent()
		{
			TypeName name = typeid(T).name();
			auto it = m_componentBitPosition.find(name);
			if (it == m_componentBitPosition.end())
				return nullptr;

			return &it->second.m_requiredComponents;
		}

		template <typename T>
		ComponentMask* const GetSustainedComponent()
		{
			TypeName name = typeid(T).name();
			auto it = m_componentBitPosition.find(name);
			if (it == m_componentBitPosition.end())
				return nullptr;

			return &it->second.m_isRequiredInComponents;
		}


		template <typename Component>
		void SetComponentBit(ComponentMask& mask, bool val) {
			size_t bitPos = GetComponentBitPosition<Component>();
			BSEECS_ASSERT(bitPos != tombstone,
				"Attempting to operate on unregistered component '" << typeid(Component).name() << "'");

			mask[bitPos] = val;
		}

		template <typename Component>
		ComponentMask::reference GetComponentBit(ComponentMask& mask) {
			size_t bitPos = GetComponentBitPosition<Component>();
			BSEECS_ASSERT(bitPos != tombstone,
				"Attempting to operate on unregistered component '" << typeid(Component).name() << "'");

			return mask[bitPos];
		}

		/*
		*  Assembles a generic mask for the given components
		*/
		template <typename... Components>
		ComponentMask GetMask() {
			ComponentMask mask;
			(SetComponentBit<Components>(mask, 1), ...); // fold expression
			return mask;
		}

		template <typename DependentComponent, typename RequiredComponent>
		void SetRequirements()
		{
			TypeName name = typeid(RequiredComponent).name();
			ComponentMask& requiredCompMask = m_componentBitPosition.find(name)->second.m_isRequiredInComponents;
			SetComponentBit<DependentComponent>(requiredCompMask, 1);
		}

		/*
		*  Broadcast to the components that I depends on, the requirement
		* To make it easy when removing it
		*/
		template <typename DependentComponent, typename... RequiredComponents>
		void BroadcastRequirements()
		{
			// Get the bit position of the component
			(SetRequirements<DependentComponent, RequiredComponents>(), ...); // fold expression
		}

	public:

		ECS() = default;

		/*
		* Retrieves reference for the specific component pool given a component name
		*/
		template <typename T>
		SparseSet<T>& GetComponentPool(bool registerIfNotFound = false)
		{
			size_t bitPos = GetComponentBitPosition<T>();

			if (bitPos == tombstone)
			{
				if (registerIfNotFound)
				{
					RegisterComponent<T>();
					bitPos = GetComponentBitPosition<T>();
				}
				BSEECS_ASSERT(registerIfNotFound,
					"Attempting to operate on unregistered component '" << typeid(T).name() << "'");
			}

			BSEECS_ASSERT(bitPos < m_componentPools.size() && bitPos >= 0,
				"(Internal): Attempting to index into m_componentPools with out of range bit position");

			// Downcast the generic pointer to the specific sparse set
			ISparseSet* genericPtr = m_componentPools[bitPos].get();
			SparseSet<T>* pool = dynamic_cast<SparseSet<T>*>(genericPtr);
			BSEECS_ASSERT(pool, "Dynamic cast failed for component pool '" << typeid(T).name() << "'");

			return *pool;
		}


		/*
		*  Creates an entity and returns the ID to refer to that entity.
		* 
		*  @param(name): 
		*  * Optional and used for debugging purposes, it
		*    shouldn't be used often since there's no optimization 
		*    in place yet for entities that share a name.
		*/
		EntityID CreateEntity(std::string_view name="") {
			EntityID id = tombstone;

			if (m_availableEntities.size() == 0) {
				BSEECS_ASSERT(m_maxEntityID < MAX_ENTITIES, "Entity limit exceeded");
				id = m_maxEntityID++;
			}
			else {
				id = m_availableEntities.back();
				m_availableEntities.pop_back();
			}

			BSEECS_ASSERT(id != tombstone, "Cannot create entity with null ID");

			if (!name.empty())
				m_entityNames[id] = name;

			BSEECS_INFO("Created entity " << ENTITY_INFO(id));
			return id;
		}

		std::string GetEntityName(EntityID id) {
			BSEECS_ASSERT_VALID_ENTITY(id);
			//BSEECS_ASSERT_ALIVE_ENTITY(id);

			auto it = m_entityNames.find(id);
			if (it == m_entityNames.end())
				return "Entity";

			return it->second;
		}

		/*
		* Deletes an active entity and its associated components.
		* - Overwrites the given entity to NULL_ENTITY.
		* 
		* This should NOT be used in the middle of a system while iterating
		* through entities, as it will remove from the list immediately. Use
		* FlagEntity(id, true) to mark an entity for deletion, and then DeleteFlagged()
		* At the end of a frame to clear all flagged entities instead.
		*/
		void DeleteEntity(EntityID& id) {
			BSEECS_ASSERT_VALID_ENTITY(id);
			//BSEECS_ASSERT_ALIVE_ENTITY(id);
			
			std::string name = GetEntityName(id);

			m_entityNames.erase(id);
			m_availableEntities.push_back(id);

			BSEECS_INFO("Deleted entity ['" << name << "', ID: " << id << "]");
			id = NULL_ENTITY;
		}

		/*
		*  Register a component with specific required components 
		*	and create a pool for it
		*/
		template <typename T, typename ...Components>
		void RegisterComponent() {
			TypeName name = typeid(T).name();
			BSEECS_ASSERT(m_componentBitPosition.find(name) == m_componentBitPosition.end(),
				"Component with name '" << name << "' already registered");
			BSEECS_ASSERT(m_componentPools.size() < MAX_COMPONENTS,
				"Exceeded max number of registered components");

			ComponentMask requiredCompMask = GetMask<Components...>();

			m_componentBitPosition.emplace(
				name
				, ComponentInfo(
					m_componentPools.size()
					, requiredCompMask
					, ComponentMask()
				)
			);
			m_componentPools.push_back(std::make_unique<SparseSet<T>>());

			// Hello I require you, so beware when you be removed
			BroadcastRequirements<T, Components...>();

			BSEECS_INFO("Registered component '" << name << "'");
		}

		/*
		*  Attaches a component to an entity
		* 
		* - AddComponent<Transform>(player, {x, y, z});
		*/
		template <typename T, typename... RequiredComponents>
		T& Add(EntityID id, T&& component={}) {
			BSEECS_ASSERT_VALID_ENTITY(id);
			//BSEECS_ASSERT_ALIVE_ENTITY(id);

			// Do this first so component pool gets registered before Has<T>()
			SparseSet<T>& pool = GetComponentPool<T>(true);

			BSEECS_ASSERT(!pool.Get(id),
				ENTITY_INFO(id) << " already has component '" << typeid(T).name() << "' added");


			ComponentMask incomingCompMask = GetMask<RequiredComponents...>();
			ComponentMask requiredCompMask = *GetRequiredComponent<T>();
			ComponentMask resultDiff = incomingCompMask ^ requiredCompMask;
			BSEECS_ASSERT(resultDiff.none(),
				typeid(T).name() << " required components mismatch");

			// What about checking and then add if not present
			bool requiredSatisfied = HasAllRequired<RequiredComponents...>(id);
			BSEECS_ASSERT(requiredSatisfied,
				ENTITY_INFO(id) << " is missing some required components ");

			BSEECS_INFO("Attached '" << typeid(T).name() << "' to " << ENTITY_INFO(id));
			return *pool.Set(id, std::move(component));
		}

		/*
		*  Retrieves the specified component for the given entity
		* 
		* - ecs.GetComponent<Transform>(player);
		*/
		template <typename T>
		T& Get(EntityID id) {
			BSEECS_ASSERT_VALID_ENTITY(id);

			SparseSet<T>& pool = GetComponentPool<T>();
			T* component = pool.Get(id);
			BSEECS_ASSERT(component,
				ENTITY_INFO(id) << " missing component 'in " << typeid(T).name() << "' pool");

			return *component;
		}

		/*
		*  Removes a component from an entity
		* 
		* - ecs.RemoveComponent<Transform>(player);
		*/
		template <typename T, typename... SustainedComponents>
		void Remove(EntityID id) {
			BSEECS_ASSERT_VALID_ENTITY(id);

			SparseSet<T>& pool = GetComponentPool<T>();
			BSEECS_ASSERT(pool.Get(id),
				ENTITY_INFO(id) << " has no component '" << typeid(T).name() << "' to remove");

			ComponentMask incomingCompMask = GetMask<SustainedComponents...>();
			ComponentMask sustainedCompMask = *GetSustainedComponent<T>();
			ComponentMask resultDiff = incomingCompMask ^ sustainedCompMask;
			BSEECS_ASSERT(resultDiff.none(),
				typeid(T).name() << " sustained components mismatch");

			// What about checking and then add if not present
			bool allSustainedRemoved = HasRemovedAll<SustainedComponents...>(id);
			BSEECS_ASSERT(allSustainedRemoved,
				ENTITY_INFO(id) << " Delete first all sustained components ");

			pool.Delete(id);
			BSEECS_INFO("Removed '" << typeid(T).name() << "' from " << ENTITY_INFO(id));
		}

		template <typename T>
		bool Has(EntityID id) {
			SparseSet<T>& pool = GetComponentPool<T>();
			return pool.Get(id) ? true : false;
		}

		template <typename... Ts>
		bool HasAll(EntityID id) {
			// Fold operator, reads as 
			// (HasComponent<Transform> && HasComponent<Physics> && HasComponent<Sprite> && ...)
			return (Has<Ts>(id) && ...);
		}

		template <typename T>
		bool HasRequired(EntityID id)
		{
			SparseSet<T>& pool = GetComponentPool<T>();
			T* hasId = pool.Get(id);

			BSEECS_ASSERT(hasId,
				ENTITY_INFO(id) << " has no REQUIRED component '" << typeid(T).name());

			return hasId;
		}

		template <typename... Ts>
		bool HasAllRequired(EntityID id)
		{
			// Fold operator, reads as 
			// (HasRequired<Transform> && HasRequired<Physics> && HasRequired<Sprite> && ...)
			return (HasRequired<Ts>(id) && ...);
		}

		template <typename T>
		bool HasRemoved(EntityID id)
		{
			SparseSet<T>& pool = GetComponentPool<T>();
			T* hasId = pool.Get(id);

			bool ret = hasId ? false : true;

			BSEECS_ASSERT(ret,
				ENTITY_INFO(id) << " has to remove component '" << typeid(T).name());

			return ret;
		}

		template <typename... Ts>
		bool HasRemovedAll(EntityID id)
		{
			// Fold operator, reads as 
			// (HasRequired<Transform> && HasRequired<Physics> && HasRequired<Sprite> && ...)
			return (HasRemoved<Ts>(id) && ...);
		}

		template <typename TDense, typename T>
		T& GetSibiling(EntityID DenseID)
		{
			EntityID EntId = GetComponentPool<TDense>().GetEntity(DenseID);
			return GetComponentPool<T>().GetRef(EntId);
		}

		template <typename T>
		T& GetSibiling(const SparseSet<T>& DensCompPool, EntityID DenseID)
		{
			EntityID EntId = DensCompPool.GetEntity(DenseID);
			return GetComponentPool<T>().GetRef(EntId);
		}

		/*
		*  Executes a passed lambda on all the entities that match the
		*  passed parameter pack.
		*
		*  Provided function should follow one of two forms:
		*  Provided function should follow one of two forms:
		*  [](Component& c1, Component& c2);
		*	The Main Component will be used to access the dense data
		*	and extract the entity ID for the others.
		*/
		template <typename MainComponent, typename ...Components, typename Func>
		void ForEach(Func&& func)
		{
			auto& compPool = GetComponentPool<MainComponent>();
			auto& dense = compPool.Data();
			//for (EntityID id : ids.Data())
			for (size_t i = 0; i < dense.size(); i++)
			{
				EntityID id = compPool.GetEntity(i);
				// This branch is for [](EntityID id, Component& c1, Component& c2);
				// constexpr denotes this is evaluated at compile time, which allows
				// the calling of func with different parameters.
				if constexpr (std::is_invocable_v<Func, EntityID, MainComponent&, Components&...>)
				{
					func(id, dense[i], GetComponentPool<Components>().GetRef(id)...);
				}

				// This branch is for [](Component& c1, Component& c2);
				else if constexpr (std::is_invocable_v<Func, MainComponent&, Components&...>)
				{
					func(dense[i], GetComponentPool<Components>().GetRef(id)...);
				}

				else
				{
					BSEECS_ASSERT(false,
						"Bad lambda provided to .ForEach(), parameter pack does not match lambda args");
				}
			}
		}
	};

	// Main comp should require all the other components
	template<typename MainComponent, typename... Components>
	class ISystem
	{
	public:
		ISystem(ECS& ecs)
			: m_Ecs(ecs), 
			m_MainComps(ecs.GetComponentPool<MainComponent>()),
			m_MainDense(ecs.GetComponentPool<MainComponent>().Data())
		{
			for (size_t i = 0; i < m_MainDense.size(); i++)
			{
				ecs.HasAllRequired<Components...>(i);
			}
		}
	protected:
		ECS& m_Ecs;
		std::vector<MainComponent>& m_MainDense;
		SparseSet<MainComponent>& m_MainComps;
	};

}

#endif