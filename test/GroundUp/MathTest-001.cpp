#include <iostream>
#include <btBulletDynamicsCommon.h>

void PrintTransform( btTransform *t )
{
	int n;
	for( n = 0; n < 3; n++ )
		printf( "   < %g %g %g %g >\n", t->getBasis()[n][0], t->getBasis()[n][1], t->getBasis()[n][2], t->getBasis()[n][3] );
	printf( "O = <%g,%g,%g>\n", t->getOrigin()[0], t->getOrigin()[1], t->getOrigin()[1] );
	printf( "\n" );
}

int main( void )
{
	{
		btTransform planeObjWorld( btQuaternion( 0.2, 0.1, 0.3, 1 ), btVector3( 3, 4, 5 ) );
		btTransform convexWorldTransform( btQuaternion( 0.2, 0.3, 0, 1 ), btVector3( 3, 4, 5 ) );
		btTransform convexInPlaneTrans;
		convexInPlaneTrans = planeObjWorld.inverse() * convexWorldTransform;
		PrintTransform( &convexInPlaneTrans );
		convexInPlaneTrans = planeObjWorld.inverseTimes( convexWorldTransform );
		PrintTransform( &convexInPlaneTrans );
		//convexInPlaneTrans = planeObjWorld( convexWorldTransform;
		//PrintTransform( &convexInPlaneTrans );
	}
	{
		btTransform planeObjWorld( btQuaternion( 0.2, 0.1, 0.3, 1 ), btVector3( 5, 2, 1 ) );
		btTransform convexWorldTransform( btQuaternion( 0.2, 0.3, 0, 1 ), btVector3( 3, 4, 5 ) );
		btTransform convexInPlaneTrans;
		convexInPlaneTrans = planeObjWorld.inverse() * convexWorldTransform;
		PrintTransform( &convexInPlaneTrans );
		convexInPlaneTrans = planeObjWorld.inverseTimes( convexWorldTransform );
		PrintTransform( &convexInPlaneTrans );
		//convexInPlaneTrans = planeObjWorld( convexWorldTransform;
		//PrintTransform( &convexInPlaneTrans );

		btQuaternion perturbeRot( btQuaternion( 0.1, 0.5, 0.25, 0.8 ) );
		perturbeRot.normalize();

		btTransform planeObjWrapTrans( btQuaternion( 0.2, 0.1, 0.3, 1 ), btVector3( 4, 7, 2 ) );
		//btTransform convexInPlaneTrans;
		convexInPlaneTrans = planeObjWrapTrans.inverse() * convexWorldTransform;
		//now perturbe the convex-world transform
		convexWorldTransform.getBasis() *= btMatrix3x3( perturbeRot );
		PrintTransform( &convexWorldTransform );
		btTransform planeInConvex;
		planeInConvex = convexWorldTransform.inverse() *planeObjWrapTrans;
		PrintTransform( &planeInConvex );
	}
	{
	}

	int pause;
	std::cin >> pause;


}

