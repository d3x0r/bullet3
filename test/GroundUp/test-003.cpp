#include <iostream>
#include <btBulletDynamicsCommon.h>


btDiscreteDynamicsWorld* world;
btRigidBody* fallingRigidBody;
btRigidBody* fallingRigidBody2;

void InitDefaultWorld( bool sloped )
{
	{
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();


		world = new btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );
	}
	{
		btCollisionShape* groundShape;
		if( sloped )
			groundShape = new btStaticPlaneShape( btVector3::Zero, (btVector3::yAxis + btVector3::xAxis).normalize() );
		else
			groundShape = new btStaticPlaneShape( btVector3::Zero, btVector3::yAxis );

		btDefaultMotionState* groundMotionState = new btDefaultMotionState();

		btRigidBody::btRigidBodyConstructionInfo*
			groundRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo( 0, groundMotionState
				, groundShape, btVector3::Zero );
		btRigidBody* groundRigidBody = new btRigidBody( *groundRigidBodyCI );
		world->addRigidBody( groundRigidBody );
	}

 	//-------------------------------------------------------

	{
		btCollisionShape* fallShape = new btBoxShape( btVector3::One );

		btVector3 origin( 0, 50, 0 );
		btTransform* init = new btTransform( btQuaternion::Identity, origin );
		btDefaultMotionState* fallMotionState = new btDefaultMotionState( *init );

		btScalar mass = 1;
		btVector3 fallInertia;
		fallShape->calculateLocalInertia( mass, fallInertia ); // fills fallInertia

		btRigidBody::btRigidBodyConstructionInfo*
			fallingRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo( mass, fallMotionState
				, fallShape, fallInertia );

		fallingRigidBody = new btRigidBody( *fallingRigidBodyCI );

		world->addRigidBody( fallingRigidBody );
	}

 	//-------------------------------------------------------

	{
		btCollisionShape* fallShape2 = new btSphereShape( 1 );

		btVector3 origin( sloped?-34:0.25, 1, 0.25 );
		btTransform* init = new btTransform( btQuaternion::Identity, origin );
		btDefaultMotionState* fallMotionState = new btDefaultMotionState( *init );

		btScalar mass = 1;
		btVector3 fallInertia;
		fallShape2->calculateLocalInertia( mass, fallInertia ); // fills fallInertia

		btRigidBody::btRigidBodyConstructionInfo*
			fallingRigidBodyCI = new btRigidBody::btRigidBodyConstructionInfo( mass, fallMotionState
				, fallShape2, fallInertia );

		fallingRigidBody2 = new btRigidBody( *fallingRigidBodyCI );

		world->addRigidBody( fallingRigidBody2 );
	}


}

int main( int argc, char **argv )
{
	InitDefaultWorld( argc > 1 );

	for( int i = 0; i < 300; i++ ) {

		if( i == 185 )
		{
			int a = 3;
		}
		world->stepSimulation( 1 / 60.f, 10 );

		btTransform trans;
		btTransform trans2;
		fallingRigidBody->getMotionState()->getWorldTransform( trans );
		fallingRigidBody2->getMotionState()->getWorldTransform( trans2 );

		std::cout << "Iteration " << i << std::endl;

		std::cout << trans.ToString( "cube orient\t", "\t\t", "cube origin\t" ) << std::endl;
		btVector3 v = fallingRigidBody->getAngularVelocity();
		std::cout << "cube Ang Vel: " << v.ToString() << std::endl;
		v = fallingRigidBody->getLinearVelocity();
		std::cout << "cube Lin Vel: " << v.ToString() << std::endl;

		std::cout << trans2.ToString( "ball orient\t", "\t\t", "ball origin\t" ) << std::endl;
		v = fallingRigidBody2->getAngularVelocity();
		std::cout << "ball Ang Vel: " << v.ToString() << std::endl;
		v = fallingRigidBody2->getLinearVelocity();
		std::cout << "ball Lin Vel: " << v.ToString() << std::endl;
	}
	int pause;
	std::cin >> pause;


}

