# attitude_lib

This is still a work in progress.

Quaternions are divided into:

	Quaternion_Base
	
	Quaternion
	
	Unit_Quaternion
	
	
The Quaternion_Base class is meant to represent quaternion numbers only. Quaternion class represents quaternions as elements of a vector space forming a division algebra. Unit_Quaternions class represents unit quaternions as a non-abelian group.

This separation is a bit strict especially for practical use, but it's straightforward to modify the implementations in here for a more practical usecase.

The Vec3 and Mat3 classes are meant to be helper classes and not full representations of euclidean vectors and GL(3,R).

TODO:
	- Finish function that are not implemented atm (for instance get_attitude_DCM())
	- Better, and perhaps more flexibility with, numerical integration.
	- Implementation of attitude filter (Madgwick, MEKF, ECF etc).
	- Finish TODO list.
