/**
 * \file EnvireOdeCollisionPlugins.cpp
 * \author Malte Langosz
 *
 */

#include "EnvireOdeCollisionPlugins.hpp"

#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>

#include <mars_interfaces/Logging.hpp>

#include <mars_ode_collision/CollisionSpace.hpp>
#include <mars_ode_collision/objects/Object.hpp>
#include <mars_ode_collision/objects/Mesh.hpp>
#include <mars_ode_collision/objects/Heightfield.hpp>


namespace mars
{
    namespace envire_ode_collision
    {
        using namespace interfaces;

        EnvireOdeCollisionPlugins::EnvireOdeCollisionPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface{theManager}
        {
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Heightfield>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Plane>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Box>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Capsule>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Cylinder>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Mesh>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Sphere>>::subscribe(ControlCenter::envireGraph.get());
        }

        EnvireOdeCollisionPlugins::~EnvireOdeCollisionPlugins()
        {}

        // TODO: this should be moved out from here
        std::shared_ptr<SubControlCenter> EnvireOdeCollisionPlugins::getControlCenter(envire::core::FrameId frame)
        {
            // search for physics interface in graph
            bool done = false;
            while(!done)
            {
                const auto& vertex = ControlCenter::envireGraph->vertex(frame);
                const auto& parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = ControlCenter::envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        const auto& it = ControlCenter::envireGraph->getItem<SubControlItem>(frame);
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

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Heightfield>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Plane>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            config["extend"]["x"] = config["size"]["x"];
            config["extend"]["y"] = config["size"]["y"];

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Box>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            config["extend"]["x"] = config["size"]["x"];
            config["extend"]["y"] = config["size"]["y"];
            config["extend"]["z"] = config["size"]["z"];

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Capsule>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Cylinder>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            config["extend"]["x"] = config["radius"];
            config["extend"]["y"] = config["length"];

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Mesh>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Sphere>>& e)
        {
            if (e.item->getTag() != "collision")
            {
                return;
            }

            auto& collidable = e.item->getData();
            auto config = collidable.getFullConfigMap();

            config["extend"]["x"] = config["radius"];

            createCollision(config, e.frame);
        }

        void EnvireOdeCollisionPlugins::createCollision(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
            auto subControl = getControlCenter(frameId);
            if (!subControl)
            {
                return;
            }

            // TODO: do we need to check the physic name here?
            //if(control->physics->getLibName() == "mars_ode_physics")
            //{

            // TODO: we will not use prefix, when we move to base envire types

            LOG_INFO("Added smurf::Collidable item: %s", frameId.c_str());
            // TODO: check that we really have the frame in the map
            const auto& vertex = ControlCenter::envireGraph->vertex(frameId);
            const auto& parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
            const auto& parentFrame = ControlCenter::envireGraph->getFrameId(parentVertex);

            LOG_INFO("collision parent Frame: %s", parentFrame.c_str());
            // LOG_ERROR("parent Frame: %s", parentFrame.c_str());
            //std::string type = e.item->getData().getTypeString();

            // TODO: why we need this? since name is set in config map, check it
            // TODO: no need default bitmask
            config["name"] = frameId;//e.item->getData().getName();
            if (!config.hasKey("bitmask"))
            {
                config["bitmask"] = std::numeric_limits<uint16_t>::max(); // == 65535
            }
            else
            {
                // TODO: for some reason bitmask after loading smurf is not a number, but string
                // need to be checked
                config["bitmask"] = static_cast<int>(config["bitmask"]);
            }

            //config["parentFrame"] = parentFrame;
            auto parent = subControl->physics->getFrame(parentFrame);

            auto* const collision = subControl->collision->createObject(config, parent);
            if(!collision)
            {
                LOG_ERROR("Error creating collision object!");
                return;
            }

            // add mesh data handling; todo: move to collsion library itself
            if(config["type"] == "mesh")
            {
                NodeData node;
                if(!config.hasKey("extend"))
                {
                    config["loadSizeFromMesh"] = true;
                    if(config.hasKey("scale"))
                    {
                        config["physicalScale"] = config["scale"];
                    }
                }
                //fprintf(stderr, "%s\n", config.toYamlString().c_str());
                node.fromConfigMap(&config, config["filePrefix"].toString());
                ControlCenter::loadCenter->loadMesh->getPhysicsFromMesh(&node);
                ((ode_collision::Mesh*)collision)->setMeshData(node.mesh);
                config["extend"]["x"] = node.ext.x();
                config["extend"]["y"] = node.ext.y();
                config["extend"]["z"] = node.ext.z();
                collision->createGeom();
            }

            if(config["type"] == "heightfield")
            {
                NodeData node;
                node.fromConfigMap(&config, config["filePrefix"].toString());

                // check physics type:
                if(node.terrain)
                {
                    if(!node.terrain->pixelData)
                    {
                        LOG_INFO("Load heightmap pixelData...");
                        ControlCenter::loadCenter->loadHeightmap->readPixelData(node.terrain);
                        if(!node.terrain->pixelData)
                        {
                            LOG_ERROR("NodeManager::addNode: could not load image for terrain");
                        }
                    }

                    ((ode_collision::Heightfield*)collision)->setTerrainStrcut(node.terrain);
                    if(!collision->createGeom())
                    {
                        LOG_ERROR("Error creating Heightfield geom!");
                        return;
                    }
                }
            }


            // TODO: check hirarchy issues with closed loops
            const auto& t = ControlCenter::envireGraph->getTransform(parentVertex, vertex);
            collision->setPosition(t.transform.translation);
            collision->setRotation(t.transform.orientation);

            // store the collision physic object in the graph
            // TODO: this is just quick implementation
            // TODO: add interface for CollisionObject how it was done for DynamicObject
            const auto collisionPtr = std::shared_ptr<ode_collision::Object>{collision};
            const auto collisionItemPtr = envire::core::Item<std::shared_ptr<ode_collision::Object>>::Ptr{new envire::core::Item<std::shared_ptr<ode_collision::Object>>{collisionPtr}};
            ControlCenter::envireGraph->addItemToFrame(frameId, collisionItemPtr);
        }

    } // end of namespace envire_ode_collision

} // end of namespace mars

DESTROY_LIB(mars::envire_ode_collision::EnvireOdeCollisionPlugins);
CREATE_LIB(mars::envire_ode_collision::EnvireOdeCollisionPlugins);
