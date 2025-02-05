#pragma once

#include "Vec.hpp"

using gmath::Vec;

namespace gphysics {
    class Bounds {
    public:
        Bounds() { Clear(); }
        Bounds( const Bounds & rhs ) : mins( rhs.mins ), maxs( rhs.maxs ) {}
        const Bounds & operator = ( const Bounds & rhs );
        ~Bounds() {}

        void Clear() { mins = Vec( 1e6, 1e6, 1e6 ); maxs = Vec( -1e6, -1e6, -1e6 ); }
        bool DoesIntersect( const Bounds & rhs ) const;
        void Expand( const Vec * pts, const int num );
        void Expand( const Vec & rhs );
        void Expand( const Bounds & rhs );

        float WidthX() const { return maxs.x - mins.x; }
        float WidthY() const { return maxs.y - mins.y; }
        float WidthZ() const { return maxs.z - mins.z; }

    public:
        Vec mins;
        Vec maxs;
    };
}