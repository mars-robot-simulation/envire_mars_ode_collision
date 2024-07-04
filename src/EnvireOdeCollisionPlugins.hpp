/**
 * \file EnvireOdeCollisionPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load collision representation based on envire items
 *
 */

#pragma once

#include <lib_manager/LibInterface.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_types/geometry/Plane.hpp>
#include <envire_types/geometry/Box.hpp>
#include <envire_types/geometry/Capsule.hpp>
#include <envire_types/geometry/Cylinder.hpp>
#include <envire_types/geometry/Mesh.hpp>
#include <envire_types/geometry/Sphere.hpp>

//TODO: add prismatic joint into base types and here

#include <mars_interfaces/sim/PhysicsInterface.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/Vector.h>


namespace mars
{
    namespace envire_ode_collision
    {
        // move the typedef to separate file
        class EnvireOdeCollisionPlugins : public lib_manager::LibInterface,
                                        public envire::core::GraphEventDispatcher,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Plane>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Box>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Capsule>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Cylinder>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Mesh>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Sphere>>

        {

        public:
            EnvireOdeCollisionPlugins(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~EnvireOdeCollisionPlugins();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string{"envire_mars_ode_collision"};
            }

            CREATE_MODULE_INFO();

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Plane>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Box>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Capsule>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Cylinder>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Mesh>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Sphere>>& e) override;

        private:
            // TODO: Move to central location
            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            void createCollision(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
        };

    } // end of namespace envire_ode_physics
} // end of namespace mars
