/**
 * \file EnvireOdeCollisionPlugins.cpp
 * \author Malte Langosz
 *
 */

#include "EnvireOdeCollisionPlugins.hpp"

//#include "PhysicsMapper.h"

#include <mars/utils/mathUtils.h>
#include <mars/utils/misc.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_interfaces/Logging.hpp>

#include <mars_ode_collision/CollisionSpace.hpp>
#include <mars_ode_collision/objects/Object.hpp>

#define SIM_CENTER_FRAME_NAME "world"
typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

namespace mars
{
    namespace envire_ode_collision
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        EnvireOdeCollisionPlugins::EnvireOdeCollisionPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface(theManager)
        {
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Box>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Capsule>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Cylinder>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Mesh>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Sphere>>::subscribe(ControlCenter::envireGraph.get());
        }

        EnvireOdeCollisionPlugins::~EnvireOdeCollisionPlugins()
        {
        }

        // TODO: this should be moved out from here
        std::shared_ptr<SubControlCenter> EnvireOdeCollisionPlugins::getControlCenter(envire::core::FrameId frame)
        {
            // search for physics interface in graph
            bool done = false;
            while(!done)
            {
                const envire::core::GraphTraits::vertex_descriptor vertex = ControlCenter::envireGraph->vertex(frame);
                envire::core::GraphTraits::vertex_descriptor parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = ControlCenter::envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        envire::core::EnvireGraph::ItemIterator<SubControlItem> it = ControlCenter::envireGraph->getItem<SubControlItem>(frame);
                        return it->getData();
                    }
                    catch (...)
                    {
                    }
                }
                else
                {
                    done = true;
                }
            }
            return nullptr;
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Box>>& e)
        {
            if (e.item->getTag() != "collision")
                return;

            envire::base_types::geometry::Box& collidable = e.item->getData();
            ConfigMap config = collidable.getFullConfigMap();

            config["extend"]["x"] = config["size"]["x"];
            config["extend"]["y"] = config["size"]["y"];
            config["extend"]["z"] = config["size"]["z"];

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Capsule>>& e)
        {
            if (e.item->getTag() != "collision")
                return;

            envire::base_types::geometry::Capsule& collidable = e.item->getData();
            ConfigMap config = collidable.getFullConfigMap();

            LOG_ERROR(" NO COLLISION IS IMPLEMENTED FOR TYPE envire::base_types::geometry::Capsule ");

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Cylinder>>& e)
        {
            if (e.item->getTag() != "collision")
                return;

            envire::base_types::geometry::Cylinder& collidable = e.item->getData();
            ConfigMap config = collidable.getFullConfigMap();

            LOG_ERROR(" NO COLLISION IS IMPLEMENTED FOR TYPE envire::base_types::geometry::Cylinder ");

            //createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Mesh>>& e)
        {
            if (e.item->getTag() != "collision")
                return;

            envire::base_types::geometry::Mesh& collidable = e.item->getData();
            ConfigMap config = collidable.getFullConfigMap();

            LOG_ERROR(" NO COLLISION IS IMPLEMENTED FOR TYPE envire::base_types::geometry::Mesh ");

            // createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Sphere>>& e)
        {
            if (e.item->getTag() != "collision")
                return;

            envire::base_types::geometry::Sphere& collidable = e.item->getData();
            ConfigMap config = collidable.getFullConfigMap();

            config["extend"]["x"] = config["radius"];

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::createCollision(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
            std::shared_ptr<interfaces::SubControlCenter> subControl = getControlCenter(frameId);
            if (subControl == nullptr)
                return;

            // TODO: do we need to check the physic name here?
            //if(control->physics->getLibName() == "mars_ode_physics")
            //{

            // TODO: we will not use prefix, when we move to base envire types

            LOG_INFO("Added smurf::Collidable item: %s", frameId.c_str());
            // todo: check that we really have the frame in the map
            const envire::core::GraphTraits::vertex_descriptor vertex = ControlCenter::envireGraph->vertex(frameId);
            envire::core::GraphTraits::vertex_descriptor parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
            envire::core::FrameId parentFrame = ControlCenter::envireGraph->getFrameId(parentVertex);

            LOG_INFO("collision parent Frame: %s", parentFrame.c_str());
            // LOG_ERROR("parent Frame: %s", parentFrame.c_str());
            //std::string type = e.item->getData().getTypeString();

            // TODO: why we need this? since name is set in config map, check it
            // TODO: no need default bitmask
            config["name"] = frameId;//e.item->getData().getName();
            if (!config.hasKey("bitmask"))
            {
                config["bitmask"] = 65535;
            } else {
                // TODO: for some reason bitmask after loading smurf is not a number, but string
                // need to be checked
                config["bitmask"] = (int)config["bitmask"];
            }

            //config["parentFrame"] = parentFrame;
            // TODO: is DynamicObject meaningfull name? should it not bw PhysicObject?
            std::shared_ptr<DynamicObject> parent = subControl->physics->getFrame(parentFrame);

            ode_collision::Object* collision = subControl->collision->createObject(config, parent);
            if(!collision)
            {
                LOG_ERROR("Error creating collision object!");
                return;
            }
            // todo: check hirarchy issues with closed loops
            envire::core::Transform t = ControlCenter::envireGraph->getTransform(parentVertex, vertex);
            collision->setPosition(t.transform.translation);
            collision->setRotation(t.transform.orientation);

            // store the collision physic object in the graph
            // TODO: this is just quick implementation
            // TODO: add interface for CollisionObject how it was done for DynamicObject
            std::shared_ptr<ode_collision::Object> collisionPtr(collision);
            envire::core::Item<std::shared_ptr<ode_collision::Object>>::Ptr collisionItemPtr(new envire::core::Item<std::shared_ptr<ode_collision::Object>>(collisionPtr));
            ControlCenter::envireGraph->addItemToFrame(frameId, collisionItemPtr);
        }

    } // end of namespace envire_ode_collision

} // end of namespace mars

DESTROY_LIB(mars::envire_ode_collision::EnvireOdeCollisionPlugins);
CREATE_LIB(mars::envire_ode_collision::EnvireOdeCollisionPlugins);
