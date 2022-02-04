//--------------------------------------------------------------------------------
// NVIDIA(R) GVDB VOXELS
// Copyright 2017, NVIDIA Corporation. 
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
//    in the documentation and/or  other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
//    from this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
// BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
// SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Version 1.0: Rama Hoetzlein, 5/1/2017
//----------------------------------------------------------------------------------
// File: cuda_gvdb_node.cuh
//
// CUDA GVDB Node header
// - Node structure
// - Node variables (CUDA or OptiX)
// - Node traversal
// - Brick helpers
//-----------------------------------------------
//#define ALIGN(x)	__align__(x)

#define ID_UNDEFI	0xFFFF
#define ID_UNDEFL	0xFFFFFFFF
//#define ID_UNDEF64	0xFFFFFFFFFFFFFFFF
#define MAX_CHANNEL  1
#define MAX_LEVELS 3
#define MAX_MIPS 30

struct VDBNode {
    int3 mPackedPosValue;
    uint mChildList;
    float4 mDensityBounds;
};

struct VDBAtlasNode {
    int3		mPos;
    int			mLeafID;
};

// mipId: 0-3 normal grid mip 0-3, 4-7 conservative grid mip 0-3, 8 temperature grid, 9 velocity grid  
struct VDBInfo {
    int			dim[MAX_LEVELS * MAX_MIPS]; // Log base 2 of lateral resolution of each node per level
    int			res[MAX_LEVELS * MAX_MIPS]; // Lateral resolution of each node per level
    float3		vdel[MAX_LEVELS * MAX_MIPS]; // How many voxels on a side a child of each level covers
    int3		noderange[MAX_LEVELS * MAX_MIPS]; // How many voxels on a side a node of each level covers
    int			nodecnt[MAX_LEVELS * MAX_MIPS]; // Total number of allocated nodes per level
    int			nodewid[MAX_LEVELS * MAX_MIPS]; // Size of a node at each level in bytes
    int			childwid[MAX_LEVELS * MAX_MIPS]; // Size of the child list per node at each level in bytes
    //char* nodelist[MAX_LEVELS]; // Pointer to each level's pool group 0 (nodes)
    //char* childlist[MAX_LEVELS]; // Pointer to each level's pool group 1 (child lists)
    StructuredBuffer<VDBNode> nodelist[MAX_LEVELS * MAX_MIPS];
    ByteAddressBuffer childlist[MAX_LEVELS * MAX_MIPS];
    //VDBAtlasNode* atlas_map; // Pointer to the atlas map (which maps from atlas to world space)
    //StructuredBuffer<VDBAtlasNode> atlas_map;
    //int3		atlas_cnt; // Number of bricks on each axis of the atlas
    //int3		atlas_res; // Total resolution in voxels of the atlas
    //int			atlas_apron; // Apron size
    //int			brick_res; // Resolution of a single brick
    //int			apron_table[8]; // Unused
    int			top_lev[MAX_MIPS]; // Top level (i.e. tree spans from voxels to level 0 to level top_lev)
    int			max_iter; // Unused
    float		epsilon; // Epsilon used for voxel ray tracing
    bool		update; // Whether this information needs to be updated from the latest volume data
    //uchar		clr_chan; // Index of the color channel for rendering color information
    uint        clr_chan;
    float3		bmin[MAX_MIPS]; // Inclusive minimum of axis-aligned bounding box in voxels
    float3		bmax[MAX_MIPS]; // Inclusive maximum of axis-aligned bounding box in voxels
    Texture3D<float>	volIn[MAX_CHANNEL * MAX_MIPS]; // Texture reference (read plus interpolation) to atlas per channel
    Texture3D<float>	volIn_part2[MAX_CHANNEL * MAX_MIPS]; // Texture reference (read plus interpolation) to atlas per channel
    Texture3D<float3>    velocityIn[2];
    Texture3D<float3>    velocityIn_part2[2];
    float superVoxelWorldSpaceDiagonalLength;
    int3 volInDimensions[MAX_CHANNEL * MAX_MIPS];
    int3 volInDimensions_part2[MAX_CHANNEL * MAX_MIPS];
    float3 invVolInDimensions[MAX_CHANNEL * MAX_MIPS];
    float3 invVolInDimensions_part2[MAX_CHANNEL * MAX_MIPS];
    //cudaSurfaceObject_t	volOut[MAX_CHANNEL]; // Surface reference (read and write) to atlas per channel

    float4x4 xform[MAX_MIPS];
    float4x4 invxform[MAX_MIPS];
    float4x4 invxrot[MAX_MIPS];

    float maxValue[MAX_MIPS];
    float invMaxValue[MAX_MIPS];
    float densityCompressScaleFactor[MAX_MIPS];
};


float3 unpackPos(int3 packed)
{
    return float3((packed.x << 16) >> 16, packed.x >> 16, (packed.y << 16) >> 16);
}

float3 unpackValue(int3 packed)
{
    return float3(packed.y >> 16, (packed.z << 16) >> 16, packed.z >> 16);
}

uint getChild(VDBInfo gvdb, int mipId, VDBNode node, int clev, int b)
{
    uint listid = node.mChildList;
    if (listid == ID_UNDEFL) return ID_UNDEFL;
    uint c = gvdb.childlist[clev + mipId * MAX_LEVELS].Load(listid * gvdb.childwid[clev + mipId * MAX_LEVELS] + 4 * b);
    return c;
}

bool isBitOn(VDBInfo gvdb, int mipId, VDBNode node, int lev, int b)
{
    return getChild(gvdb, mipId, node, lev, b) != ID_UNDEFL;
}

// get node at a specific level and pool index
VDBNode getNode(VDBInfo gvdb, int mipId, int lev, int n)
{
    VDBNode node = gvdb.nodelist[lev + mipId * MAX_LEVELS][n]; // TODO: remove bitmasks
    return node;
}

// get node at a specific level and pool index
VDBNode getNode(VDBInfo gvdb, int mipId, int lev, int n, inout float3 vmin)
{
    // TODO: check node size is equal to VDBNode (nodewid)
    VDBNode node = gvdb.nodelist[lev + mipId * MAX_LEVELS][n];
    vmin = unpackPos(node.mPackedPosValue);
    return node;
}

// iteratively find the leaf node at the given position
VDBNode getNode(VDBInfo gvdb, int mipId, int lev, int start_id, float3 pos, inout uint node_id)
{
    float3 vmin, vmax;
    int3 p;
    int b;
    node_id = ID_UNDEFL;

    VDBNode node = getNode(gvdb, mipId, lev, start_id, vmin);		// get starting node

    int globalLev = lev + mipId * MAX_LEVELS;

    for (; lev > 0; )
    {
        vmax = vmin + float3(gvdb.noderange[globalLev]);
        if (any((pos < vmin) || (pos >= vmax)))
        {
            node_id = ID_UNDEFL;
            return {};
        }
        p = int3((pos - vmin) / gvdb.vdel[globalLev]);		// check child bit
        b = (((p.z << gvdb.dim[globalLev]) + p.y) << gvdb.dim[globalLev]) + p.x;

        node_id = getChild(gvdb, mipId, node, lev, b);				// get next node_id
        lev--;
        globalLev--;

        if (node_id != ID_UNDEFL)
        {
            node = getNode(gvdb, mipId, lev, node_id, vmin);
        }
        else return {};
    }
    return node;
}


// iteratively find the leaf node at the given position
VDBNode getNodeUnrolledThreeLevel(VDBInfo gvdb, int mipId, int lev, int start_id, float3 pos, inout uint node_id)
{
    float3 vmin, vmax;
    int3 p;
    int b;
    node_id = ID_UNDEFL;

    VDBNode node = getNode(gvdb, mipId, lev, start_id, vmin);		// get starting node

    {
        vmax = vmin + 4096.f;
        if (any((pos < vmin) || (pos >= vmax)))
        {
            node_id = ID_UNDEFL;
            return {};
        }

        p = int3(pos - vmin) / 128;
        b = (((p.z << 5) + p.y) << 5) + p.x;

        node_id = getChild(gvdb, mipId, node, 2, b);				// get next node_id
        if (node_id != ID_UNDEFL)
        {
            node = getNode(gvdb, mipId, 1, node_id, vmin);
        }
        else return {};
    }

    {
        vmax = vmin + 128.f;
        if (any((pos < vmin) || (pos >= vmax)))
        {
            node_id = ID_UNDEFL;
            return {};
        }

        p = int3(pos - vmin) / 8;
        b = (((p.z << 4) + p.y) << 4) + p.x;

        node_id = getChild(gvdb, mipId, node, 1, b);				// get next node_id
        if (node_id != ID_UNDEFL)
        {
            node = getNode(gvdb, mipId, 0, node_id, vmin);
        }
        else return {};
    }

    return node;
}

// iteratively find the leaf node at the given position
VDBNode getNodeUnrolledTwoLevel(VDBInfo gvdb, int mipId, int lev, int start_id, float3 pos, inout uint node_id)
{
    float3 vmin, vmax;
    int3 p;
    int b;
    node_id = ID_UNDEFL;

    VDBNode node = getNode(gvdb, mipId, lev, start_id, vmin);		// get starting node

    {
        vmax = vmin + 128.f;
        if (any((pos < vmin) || (pos >= vmax)))
        {
            node_id = ID_UNDEFL;
            return {};
        }

        p = int3(pos - vmin) / 8;
        b = (((p.z << 4) + p.y) << 4) + p.x;

        node_id = getChild(gvdb, mipId, node, 1, b);				// get next node_id
        if (node_id != ID_UNDEFL)
        {
            node = getNode(gvdb, mipId, 0, node_id, vmin);
        }
        else return {};
    }

    return node;
}

// Gets the node pointer, atlas-space AABB minimum (not including apron), index-space AABB minimum, and voxel size of
// the brick (leaf node) containing the index-space position `pos`.
// `offs` is the brick's atlas-space AABB minimum (i.e. mValue. This does not include apron.)
// `vmin` is the brick's index-space AABB minimum
VDBNode getNodeAtPoint(VDBInfo gvdb, int mipId, float3 pos, inout float3 offs, inout float3 vmin, inout uint node_id)
{
    // iteratively get node at world point (works for all cases)
    //VDBNode node = getNode(gvdb, mipId, gvdb.top_lev[mipId], 0, pos, node_id);

    // optimization: doesn't work if the GVDB only has one level
    VDBNode node;
    if (gvdb.top_lev[mipId] == 2)
        node = getNodeUnrolledThreeLevel(gvdb, mipId, 2, 0, pos, node_id);
    else
        node = getNodeUnrolledTwoLevel(gvdb, mipId, 1, 0, pos, node_id);

    if (node_id == ID_UNDEFL) return {};

    // compute node bounding box
    vmin = unpackPos(node.mPackedPosValue);
    offs = unpackValue(node.mPackedPosValue);

    return node;
}
