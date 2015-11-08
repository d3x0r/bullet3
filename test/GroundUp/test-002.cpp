#include <iostream>
#include <btBulletDynamicsCommon.h>


btDiscreteDynamicsWorld* world;
btRigidBody* fallingRigidBody;

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


	{
		btCollisionShape* fallShape = new btSphereShape( 1 );

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


}

int main( int argc, char **argv )
{
	InitDefaultWorld( argc > 1 );

	for( int i = 0; i < 300; i++ ) {

		if( i == 188 )
		{
			int a = 3;
		}
		world->stepSimulation( 1 / 60.f, 10 );

		btTransform trans;
		fallingRigidBody->getMotionState()->getWorldTransform( trans );

		std::cout << "Iteration " << i << std::endl;

		std::cout << trans.ToString( "ball orient\t", "\t\t", "ball origin\t" ) << std::endl;
		btVector3 v = fallingRigidBody->getAngularVelocity();
		std::cout << "ball Ang Vel: " << v.ToString() << std::endl;
		v = fallingRigidBody->getLinearVelocity();
		std::cout << "ball Lin Vel: " << v.ToString() << std::endl;
	}
	int pause;
	std::cin >> pause;


}

