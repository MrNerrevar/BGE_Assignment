#include "Appa.h"
using namespace BGE;


Appa::Appa(shared_ptr<PhysicsFactory> _physicsFactory)
{
	physicsFactory = _physicsFactory;
}

Appa::Appa()
{

}

Appa::~Appa(void)
{

}

bool Appa::Initialise()
{
	return true;
}

shared_ptr<PhysicsController> Appa::CreateAppa(glm::vec3 position, float size)
{
	shared_ptr<PhysicsController> appa = CreateHead(position, 2.8);


	glm::vec3 gap = glm::vec3(0, 0, size);

	
	/*for (int i = 0; i < numSections; i++)
	{
		position += gap;
		sectionToConnect = CreateTorso(position, appa, sectionWidth, sectionHeight, sectionDepth);
		limbs.push_back(sectionToConnect);
	}*/

	for (int i = 0; i < 2; i++)
	{	
		position += gap;
		appa = CreateTorso(position, 3.5, appa);
		limbs.push_back(appa);
	}
	
	// Left Legs
	glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	CreateLegL1(legPosLeft1, 0.2, 4, appa);

	glm::vec3 legPosLeft2 = position + glm::vec3(-3.5f, -1.5, -3);
	CreateLegL2(legPosLeft2, 0.2, 4, appa);

	glm::vec3 legPosLeft3 = position + glm::vec3(-3.5f, -1.5, -0.3);
	CreateLegL3(legPosLeft3, 0.2, 4, appa);

	// Right Legs
	glm::vec3 legPosRight1 = position + glm::vec3(3.5f, -1.5, -5.7);
	CreateLegR1(legPosRight1, 0.2, 4, appa);

	glm::vec3 legPosRight2 = position + glm::vec3(3.5f, -1.5, -3);
	CreateLegR2(legPosRight2, 0.2, 4, appa);

	glm::vec3 legPosRight3 = position + glm::vec3(3.5f, -1.5, -0.3);
	CreateLegR3(legPosRight3, 0.2, 4, appa);

	glm::vec3 tailGap = glm::vec3(0, 1.5, size);
	glm::vec3 tailPos = position + tailGap;
	for (int j = 0; j < 1; j++)
	{
		CreateTail(tailPos, 5, appa);
		tailPos += gap + glm::vec3(0, 0, 0.3);
		limbs.push_back(appa);
	}

	return appa;
}

shared_ptr<PhysicsController> Appa::CreateHead(glm::vec3 position, float size)
{
	shared_ptr<PhysicsController> head = physicsFactory->CreateSphere(size, position, glm::quat(), false, true);

	return head;
}

shared_ptr<PhysicsController> Appa::CreateTorso(glm::vec3 position, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> torso = physicsFactory->CreateSphere(3.3, position, glm::quat(), false, true);
	limbs.push_back(torso);

	btHingeConstraint * hinge = new btHingeConstraint(
		*torso->rigidBody,
		*appa->rigidBody, 
		btVector3(0, 0, -4), 
		btVector3(0, 0, 2), 
		btVector3(1, 0, 0),
		btVector3(1, 0, 0), 
		true
	);
	hinge->setDbgDrawSize(btScalar(5.f));
	hinge->setLimit(btScalar(-0.20f), btScalar(0.20f));

	physicsFactory->dynamicsWorld->addConstraint(hinge);

	limbs.push_back(torso);

	return torso;
}

shared_ptr<PhysicsController> Appa::CreateTail(glm::vec3 position, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> tail = physicsFactory->CreateBox(size*0.7, size/5, size*1.7, position, glm::quat(), false , true);
	limbs.push_back(tail);
	btHingeConstraint * hinge = new btHingeConstraint( 
		*tail->rigidBody, 
		*appa->rigidBody,
		btVector3(0, -1.5, -3), 
		btVector3(0, 1, 4), 
		btVector3(1, 0, 0), 
		btVector3(1, 0, 0), 
		true
	);
	hinge->setLimit(btScalar(-0.50f), btScalar(0.50f));

	physicsFactory->dynamicsWorld->addConstraint(hinge);
	

	return tail;
}


void Appa::CreateLegL1(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCylinder(1, size, position, glm::quat(), false, true);

	//glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	btHingeConstraint * hinge = new btHingeConstraint(
		*appa->rigidBody,
		*leg->rigidBody,
		btVector3(-3.5, 1, -5.7),
		btVector3(0, 3, 0),
		btVector3(1, 0, 0),
		btVector3(1, 0, 0),
		true
	);
	hinge->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>());
	physicsFactory->dynamicsWorld->addConstraint(hinge);
}

void Appa::CreateLegL2(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCylinder(1, size, position, glm::quat(), false, true);

	//glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	btHingeConstraint * hinge = new btHingeConstraint(
		*appa->rigidBody,
		*leg->rigidBody,
		btVector3(-3.5, 1, -3),
		btVector3(0, 3, 0),
		btVector3(1, 0, 0),
		btVector3(1, 0, 0),
		true
	);
	hinge->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>());
	physicsFactory->dynamicsWorld->addConstraint(hinge);
}

void Appa::CreateLegL3(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCylinder(1, size, position, glm::quat(), false, true);

	//glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	btHingeConstraint * hinge = new btHingeConstraint(
		*appa->rigidBody,
		*leg->rigidBody,
		btVector3(-3.5, 1, -0.3),
		btVector3(0, 3, 0),
		btVector3(1, 0, 0),
		btVector3(1, 0, 0),
		true
	);
	hinge->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>());
	physicsFactory->dynamicsWorld->addConstraint(hinge);
}


void Appa::CreateLegR1(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCylinder(1, size, position, glm::quat(), false, true);

	//glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	btHingeConstraint * hinge = new btHingeConstraint(
		*appa->rigidBody,
		*leg->rigidBody,
		btVector3(3.5, 1, -5.7),
		btVector3(0, 3, 0),
		btVector3(1, 0, 0),
		btVector3(1, 0, 0),
		true
	);
	hinge->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>());
	physicsFactory->dynamicsWorld->addConstraint(hinge);
}

void Appa::CreateLegR2(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCylinder(1, size, position, glm::quat(), false, true);

	//glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	btHingeConstraint * hinge = new btHingeConstraint(
		*appa->rigidBody,
		*leg->rigidBody,
		btVector3(3.5, 1, -3),
		btVector3(0, 3, 0),
		btVector3(1, 0, 0),
		btVector3(1, 0, 0),
		true
	);
	hinge->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>());
	physicsFactory->dynamicsWorld->addConstraint(hinge);
}

void Appa::CreateLegR3(glm::vec3 position, float radius, float size, shared_ptr<PhysicsController> appa)
{
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCylinder(1, size, position, glm::quat(), false, true);

	//glm::vec3 legPosLeft1 = position + glm::vec3(-3.5f, -1.5, -5.7);
	btHingeConstraint * hinge = new btHingeConstraint(
		*appa->rigidBody,
		*leg->rigidBody,
		btVector3(3.5, 1, -0.3),
		btVector3(0, 3, 0),
		btVector3(1, 0, 0),
		btVector3(1, 0, 0),
		true
	);
	hinge->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>());
	physicsFactory->dynamicsWorld->addConstraint(hinge);
}


void Appa::Update(float timeDelta)
{
	GameComponent::Update(timeDelta);
}
