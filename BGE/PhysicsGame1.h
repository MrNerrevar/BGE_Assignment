#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include "Appa.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class PhysicsGame1 :
		public Game
	{
	private:
		shared_ptr<Appa> appa;
	public:
		PhysicsGame1(void);
		~PhysicsGame1(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall();		
	};
}
