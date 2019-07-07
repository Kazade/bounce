/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_GARMENT_CLOTH_MESH_H
#define B3_GARMENT_CLOTH_MESH_H

#include <bounce/cloth/cloth_mesh.h>

struct b3GarmentMesh;

// This mesh structure represents a cloth mesh that can be set from 2D a garment mesh.
// The mesh is set in the xy plane (z = 0).
struct b3GarmentClothMesh : public b3ClothMesh
{
	b3GarmentClothMesh();
	~b3GarmentClothMesh();

	// Set this mesh from a 2D garment mesh.
	void Set(const b3GarmentMesh* garment);
};

#endif