#pragma once
#include "GameComponent.h"
#include "PhysicsFactory.h"
#include "PhysicsController.h"

namespace BGE
{
	class Appa :
		public GameComponent
	{
	private:
		shared_ptr<PhysicsController> CreateHead(glm::vec3 position, float size);
		shared_ptr<PhysicsController> CreateTorso(glm::vec3 position, float size, shared_ptr<PhysicsController> appa);
		shared_ptr<PhysicsController> CreateTail(glm::vec3 position, float size, shared_ptr<PhysicsController> appa);
		void CreateLegL1(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa);
		void CreateLegL2(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa);
		void CreateLegL3(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa);
		void CreateLegR1(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa);
		void CreateLegR2(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa);
		void CreateLegR3(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa);
		shared_ptr<PhysicsFactory> physicsFactory;

		vector<shared_ptr<PhysicsController>> limbs;
	public:
		Appa(shared_ptr<PhysicsFactory> _physicsFactory);
		Appa();
		~Appa(void);
		bool Initialise();
		void Update(float timeDelta);
		shared_ptr<PhysicsController> CreateAppa(glm::vec3 position, float size);
	};
}
