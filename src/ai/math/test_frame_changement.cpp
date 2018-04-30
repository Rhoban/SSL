#include <gtest/gtest.h>

#include <debug.h>
#include "frame_changement.h"
#include "vector.h"

#define EPSILON 0.00001

bool are_equal(
    const rhoban_geometry::Point & v1, const rhoban_geometry::Point & v2 
){
    return norm( v1-v2 ) < EPSILON;
}

bool are_equal(
    const Vector2d & v1, const Vector2d & v2 
){
    return norm( v1-v2 ) < EPSILON;
}

bool are_equal(
    const Eigen::Vector2d & v1, const Eigen::Vector2d & v2 
){
    return ( v1-v2 ).norm() < EPSILON;
}

TEST(test_frame_changement, constructor){
    Frame_changement fc;

    Vector2d vz(0.0, 0.0);
    Vector2d vi(1.0, 0.0);
    Vector2d vj(0.0, 1.0);
    
    rhoban_geometry::Point pz(0.0, 0.0);
    rhoban_geometry::Point pi(1.0, 0.0);
    rhoban_geometry::Point pj(0.0, 1.0);

    EXPECT_TRUE( are_equal(vz, fc.to_basis(vz)) );
    EXPECT_TRUE( are_equal(vi, fc.to_basis(vi)) );
    EXPECT_TRUE( are_equal(vj, fc.to_basis(vj)) );
    EXPECT_TRUE( are_equal(vz, fc.from_basis(vz)) );
    EXPECT_TRUE( are_equal(vi, fc.from_basis(vi)) );
    EXPECT_TRUE( are_equal(vj, fc.from_basis(vj)) );
    EXPECT_TRUE( are_equal(pz, fc.to_frame(pz)) );
    EXPECT_TRUE( are_equal(pi, fc.to_frame(pi)) );
    EXPECT_TRUE( are_equal(pj, fc.to_frame(pj)) );
    EXPECT_TRUE( are_equal(pz, fc.from_frame(pz)) );
    EXPECT_TRUE( are_equal(pi, fc.from_frame(pi)) );
    EXPECT_TRUE( are_equal(pj, fc.from_frame(pj)) );
}

TEST(test_frame_changement, identity){
    Frame_changement fc;
    fc.set_frame(
        rhoban_geometry::Point(0.0, 0.0),
        Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
    );

    Vector2d vz(0.0, 0.0);
    Vector2d vi(1.0, 0.0);
    Vector2d vj(0.0, 1.0);
    
    rhoban_geometry::Point pz(0.0, 0.0);
    rhoban_geometry::Point pi(1.0, 0.0);
    rhoban_geometry::Point pj(0.0, 1.0);

    EXPECT_TRUE( are_equal(vz, fc.to_basis(vz)) );
    EXPECT_TRUE( are_equal(vi, fc.to_basis(vi)) );
    EXPECT_TRUE( are_equal(vj, fc.to_basis(vj)) );
    EXPECT_TRUE( are_equal(vz, fc.from_basis(vz)) );
    EXPECT_TRUE( are_equal(vi, fc.from_basis(vi)) );
    EXPECT_TRUE( are_equal(vj, fc.from_basis(vj)) );
    EXPECT_TRUE( are_equal(pz, fc.to_frame(pz)) );
    EXPECT_TRUE( are_equal(pi, fc.to_frame(pi)) );
    EXPECT_TRUE( are_equal(pj, fc.to_frame(pj)) );
    EXPECT_TRUE( are_equal(pz, fc.from_frame(pz)) );
    EXPECT_TRUE( are_equal(pi, fc.from_frame(pi)) );
    EXPECT_TRUE( are_equal(pj, fc.from_frame(pj)) );
}

TEST(test_frame_changement, translation){
    Frame_changement fc;
    fc.set_frame(
        rhoban_geometry::Point(1.0, 2.0),
        Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
    );

    Vector2d vz(0.0, 0.0);
    Vector2d vi(1.0, 0.0);
    Vector2d vj(0.0, 1.0);
    
    rhoban_geometry::Point pz(0.0, 0.0);
    rhoban_geometry::Point pi(1.0, 0.0);
    rhoban_geometry::Point pj(0.0, 1.0);

    EXPECT_TRUE( are_equal(vz, fc.to_basis(vz)) );
    EXPECT_TRUE( are_equal(vi, fc.to_basis(vi)) );
    EXPECT_TRUE( are_equal(vj, fc.to_basis(vj)) );
    EXPECT_TRUE( are_equal(vz, fc.from_basis(vz)) );
    EXPECT_TRUE( are_equal(vi, fc.from_basis(vi)) );
    EXPECT_TRUE( are_equal(vj, fc.from_basis(vj)) );
    EXPECT_TRUE( are_equal(rhoban_geometry::Point(-1.0,-2.0), fc.to_frame(pz)) );
    EXPECT_TRUE( are_equal(rhoban_geometry::Point(0.0,-2.0), fc.to_frame(pi)) );
    EXPECT_TRUE( are_equal(rhoban_geometry::Point(-1.0,-1.0), fc.to_frame(pj)) );
    EXPECT_TRUE( are_equal(rhoban_geometry::Point(1.0,2.0), fc.from_frame(pz)) );
    EXPECT_TRUE( are_equal(rhoban_geometry::Point(2.0,2.0), fc.from_frame(pi)) );
    EXPECT_TRUE( are_equal(rhoban_geometry::Point(1.0,3.0), fc.from_frame(pj)) );
}


TEST(test_frame_changement, rotation){
    Frame_changement fc;
    double c = .5;
    double s = std::sqrt(3.0)/2.0;
    fc.set_frame(
        rhoban_geometry::Point(0.0, 0.0),
        Vector2d(c, s), Vector2d(-s, c)
    );

    Vector2d vz(0.0, 0.0);
    Vector2d vi(1.0, 0.0);
    Vector2d vj(0.0, 1.0);

    Vector2d vfi(c, s);
    Vector2d vfj(-s, c);
    Vector2d vti(c, -s);
    Vector2d vtj(s, c);
    
    rhoban_geometry::Point pz(0.0, 0.0);
    rhoban_geometry::Point pi(1.0, 0.0);
    rhoban_geometry::Point pj(0.0, 1.0);

    rhoban_geometry::Point pfi(c, s);
    rhoban_geometry::Point pfj(-s, c);
    rhoban_geometry::Point pti(c, -s);
    rhoban_geometry::Point ptj(s, c);


    EXPECT_TRUE( are_equal(vz, fc.to_basis(vz)) );
    EXPECT_TRUE( are_equal(vti, fc.to_basis(vi)) );
    EXPECT_TRUE( are_equal(vtj, fc.to_basis(vj)) );
    EXPECT_TRUE( are_equal(vz, fc.from_basis(vz)) );
    EXPECT_TRUE( are_equal(vfi, fc.from_basis(vi)) );
    EXPECT_TRUE( are_equal(vfj, fc.from_basis(vj)) );
    EXPECT_TRUE( are_equal(pz, fc.to_frame(pz)) );
    EXPECT_TRUE( are_equal(pti, fc.to_frame(pi)) );
    EXPECT_TRUE( are_equal(ptj, fc.to_frame(pj)) );
    EXPECT_TRUE( are_equal(pz, fc.from_frame(pz)) );
    EXPECT_TRUE( are_equal(pfi, fc.from_frame(pi)) );
    EXPECT_TRUE( are_equal(pfj, fc.from_frame(pj)) );
}


template <typename VECTOR_CLASS> double X(const VECTOR_CLASS & v );
template <> double X<Vector2d>(const Vector2d & v ){ return v.getX(); }
template <> double X<rhoban_geometry::Point>(const rhoban_geometry::Point & p ){ return p.getX(); }
template <> double X<Eigen::Vector2d>(const Eigen::Vector2d & v ){ return v[0]; }

template <typename VECTOR_CLASS> double Y(const VECTOR_CLASS & v );
template <> double Y<Vector2d>(const Vector2d & v ){ return v.getY(); }
template <> double Y<rhoban_geometry::Point>(const rhoban_geometry::Point & p ){ return p.getY(); }
template <> double Y<Eigen::Vector2d>(const Eigen::Vector2d & v ){ return v[1]; }


TEST(test_frame_changement, all){
    Frame_changement fc;
    double c = .5;
    double s = std::sqrt(3.0)/2.0;
    fc.set_frame(
        rhoban_geometry::Point(1.0, 2.0),
        Vector2d(c, s), Vector2d(-s, c)
    );

    Vector2d vz(0.0, 0.0);
    Vector2d vi(1.0, 0.0);
    Vector2d vj(0.0, 1.0);

    Vector2d vfz(0.0, 0.0);
    Vector2d vfi(c, s);
    Vector2d vfj(-s, c);
    Vector2d vtz(0, 0);
    Vector2d vti(c, -s);
    Vector2d vtj(s, c);

    Vector2d vtr(1.0, 2.0);


    
    rhoban_geometry::Point pz(0.0, 0.0);
    rhoban_geometry::Point pi(1.0, 0.0);
    rhoban_geometry::Point pj(0.0, 1.0);

    rhoban_geometry::Point pfz(0.0, 0.0);
    rhoban_geometry::Point pfi(c, s);
    rhoban_geometry::Point pfj(-s, c);
    rhoban_geometry::Point ptz(0, 0);
    rhoban_geometry::Point pti(c, -s);
    rhoban_geometry::Point ptj(s, c);

    rhoban_geometry::Point ptr(1.0, 2.0);


    rhoban_geometry::Point f_pfz = ptr;
    rhoban_geometry::Point f_pfi = ptr+pfi;
    rhoban_geometry::Point f_pfj = ptr+pfj;
    rhoban_geometry::Point puz = -ptr;
    rhoban_geometry::Point pui = pi-ptr;
    rhoban_geometry::Point puj = pj-ptr;
    rhoban_geometry::Point f_ptz( X(puz)*c + Y(puz)*s , X(puz)*(-s) + Y(puz)*c );
    rhoban_geometry::Point f_pti( X(pui)*c + Y(pui)*s , X(pui)*(-s) + Y(pui)*c );
    rhoban_geometry::Point f_ptj( X(puj)*c + Y(puj)*s , X(puj)*(-s) + Y(puj)*c );
    

    EXPECT_TRUE( are_equal(vtz, fc.to_basis(vz)) );
    EXPECT_TRUE( are_equal(vti, fc.to_basis(vi)) );
    EXPECT_TRUE( are_equal(vtj, fc.to_basis(vj)) );
    EXPECT_TRUE( are_equal(vfz, fc.from_basis(vz)) );
    EXPECT_TRUE( are_equal(vfi, fc.from_basis(vi)) );
    EXPECT_TRUE( are_equal(vfj, fc.from_basis(vj)) );
    EXPECT_TRUE( are_equal(f_ptz, fc.to_frame(pz)) );
    EXPECT_TRUE( are_equal(f_pti, fc.to_frame(pi)) );
    EXPECT_TRUE( are_equal(f_ptj, fc.to_frame(pj)) );
    EXPECT_TRUE( are_equal(f_pfz, fc.from_frame(pz)) );
    EXPECT_TRUE( are_equal(f_pfi, fc.from_frame(pi)) );
    EXPECT_TRUE( are_equal(f_pfj, fc.from_frame(pj)) );
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
