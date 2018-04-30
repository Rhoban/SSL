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
    const Eigen::Vector2d & v1, const Eigen::Vector2d & v2 
){
    return ( v1-v2 ).norm() < EPSILON;
}

template <typename VECTOR_CLASS>
void check_constructor(){
    Frame_changement fc;
    VECTOR_CLASS z(0.0, 0.0);
    VECTOR_CLASS i(1.0, 0.0);
    VECTOR_CLASS j(0.0, 1.0);
    EXPECT_TRUE( are_equal(z, fc.to_basis(z)) );
    EXPECT_TRUE( are_equal(i, fc.to_basis(i)) );
    EXPECT_TRUE( are_equal(j, fc.to_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_basis(z)) );
    EXPECT_TRUE( are_equal(i, fc.from_basis(i)) );
    EXPECT_TRUE( are_equal(j, fc.from_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.to_frame(z)) );
    EXPECT_TRUE( are_equal(i, fc.to_frame(i)) );
    EXPECT_TRUE( are_equal(j, fc.to_frame(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_frame(z)) );
    EXPECT_TRUE( are_equal(i, fc.from_frame(i)) );
    EXPECT_TRUE( are_equal(j, fc.from_frame(j)) );
}

TEST(test_frame_changement, constructor){
    check_constructor<Vector2d>();
    check_constructor<rhoban_geometry::Point>();
    check_constructor<Eigen::Vector2d>();
}

template <typename VECTOR_CLASS>
void check_identity(){
    Frame_changement fc;
    fc.set_frame(
        rhoban_geometry::Point(0.0, 0.0),
        Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
    );
    VECTOR_CLASS z(0.0, 0.0);
    VECTOR_CLASS i(1.0, 0.0);
    VECTOR_CLASS j(0.0, 1.0);
    EXPECT_TRUE( are_equal(z, fc.to_basis(z)) );
    EXPECT_TRUE( are_equal(i, fc.to_basis(i)) );
    EXPECT_TRUE( are_equal(j, fc.to_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_basis(z)) );
    EXPECT_TRUE( are_equal(i, fc.from_basis(i)) );
    EXPECT_TRUE( are_equal(j, fc.from_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.to_frame(z)) );
    EXPECT_TRUE( are_equal(i, fc.to_frame(i)) );
    EXPECT_TRUE( are_equal(j, fc.to_frame(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_frame(z)) );
    EXPECT_TRUE( are_equal(i, fc.from_frame(i)) );
    EXPECT_TRUE( are_equal(j, fc.from_frame(j)) );
}

TEST(test_frame_changement, identity){
    check_identity<Vector2d>();
    check_identity<rhoban_geometry::Point>();
    check_identity<Eigen::Vector2d>();
}

template <typename VECTOR_CLASS>
void check_translation(){
    Frame_changement fc;
    fc.set_frame(
        rhoban_geometry::Point(1.0, 2.0),
        Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)
    );
    VECTOR_CLASS z(0.0, 0.0);
    VECTOR_CLASS i(1.0, 0.0);
    VECTOR_CLASS j(0.0, 1.0);
    EXPECT_TRUE( are_equal(z, fc.to_basis(z)) );
    EXPECT_TRUE( are_equal(i, fc.to_basis(i)) );
    EXPECT_TRUE( are_equal(j, fc.to_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_basis(z)) );
    EXPECT_TRUE( are_equal(i, fc.from_basis(i)) );
    EXPECT_TRUE( are_equal(j, fc.from_basis(j)) );
    EXPECT_TRUE( are_equal(VECTOR_CLASS(-1.0,-2.0), fc.to_frame(z)) );
    EXPECT_TRUE( are_equal(VECTOR_CLASS(0.0,-2.0), fc.to_frame(i)) );
    EXPECT_TRUE( are_equal(VECTOR_CLASS(-1.0,-1.0), fc.to_frame(j)) );
    EXPECT_TRUE( are_equal(VECTOR_CLASS(1.0,2.0), fc.from_frame(z)) );
    EXPECT_TRUE( are_equal(VECTOR_CLASS(2.0,2.0), fc.from_frame(i)) );
    EXPECT_TRUE( are_equal(VECTOR_CLASS(1.0,3.0), fc.from_frame(j)) );
}

TEST(test_frame_changement, translation){
    check_translation<Vector2d>();
    check_translation<rhoban_geometry::Point>();
    check_translation<Eigen::Vector2d>();
}

template <typename VECTOR_CLASS>
void check_rotation(){
    Frame_changement fc;
    double c = .5;
    double s = std::sqrt(3.0)/2.0;
    fc.set_frame(
        rhoban_geometry::Point(0.0, 0.0),
        Vector2d(c, s), Vector2d(-s, c)
    );
    VECTOR_CLASS z(0.0, 0.0);
    VECTOR_CLASS i(1.0, 0.0);
    VECTOR_CLASS j(0.0, 1.0);
    VECTOR_CLASS fi(c, s);
    VECTOR_CLASS fj(-s, c);
    VECTOR_CLASS ti(c, -s);
    VECTOR_CLASS tj(s, c);
    EXPECT_TRUE( are_equal(z, fc.to_basis(z)) );
    EXPECT_TRUE( are_equal(ti, fc.to_basis(i)) );
    EXPECT_TRUE( are_equal(tj, fc.to_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_basis(z)) );
    EXPECT_TRUE( are_equal(fi, fc.from_basis(i)) );
    EXPECT_TRUE( are_equal(fj, fc.from_basis(j)) );
    EXPECT_TRUE( are_equal(z, fc.to_frame(z)) );
    EXPECT_TRUE( are_equal(ti, fc.to_frame(i)) );
    EXPECT_TRUE( are_equal(tj, fc.to_frame(j)) );
    EXPECT_TRUE( are_equal(z, fc.from_frame(z)) );
    EXPECT_TRUE( are_equal(fi, fc.from_frame(i)) );
    EXPECT_TRUE( are_equal(fj, fc.from_frame(j)) );
}

TEST(test_frame_changement, rotation){
    check_rotation<Vector2d>();
    check_rotation<rhoban_geometry::Point>();
    check_rotation<Eigen::Vector2d>();
}

template <typename VECTOR_CLASS> double X(const VECTOR_CLASS & v );
template <> double X<Vector2d>(const Vector2d & v ){ return v.getX(); }
template <> double X<Eigen::Vector2d>(const Eigen::Vector2d & v ){ return v[0]; }

template <typename VECTOR_CLASS> double Y(const VECTOR_CLASS & v );
template <> double Y<Vector2d>(const Vector2d & v ){ return v.getY(); }
template <> double Y<Eigen::Vector2d>(const Eigen::Vector2d & v ){ return v[1]; }


template <typename VECTOR_CLASS>
void check_all(){
    Frame_changement fc;
    double c = .5;
    double s = std::sqrt(3.0)/2.0;
    fc.set_frame(
        rhoban_geometry::Point(1.0, 2.0),
        Vector2d(c, s), Vector2d(-s, c)
    );
    VECTOR_CLASS z(0.0, 0.0);
    VECTOR_CLASS i(1.0, 0.0);
    VECTOR_CLASS j(0.0, 1.0);

    VECTOR_CLASS fz(0.0, 0.0);
    VECTOR_CLASS fi(c, s);
    VECTOR_CLASS fj(-s, c);
    VECTOR_CLASS tz(0, 0);
    VECTOR_CLASS ti(c, -s);
    VECTOR_CLASS tj(s, c);

    VECTOR_CLASS tr(1.0, 2.0);

    VECTOR_CLASS f_fz = tr;
    VECTOR_CLASS f_fi = tr+fi;
    VECTOR_CLASS f_fj = tr+fj;
    VECTOR_CLASS uz = -tr;
    VECTOR_CLASS ui = i-tr;
    VECTOR_CLASS uj = j-tr;
    VECTOR_CLASS f_tz( X(uz)*c + Y(uz)*s , X(uz)*(-s) + Y(uz)*c );
    VECTOR_CLASS f_ti( X(ui)*c + Y(ui)*s , X(ui)*(-s) + Y(ui)*c );
    VECTOR_CLASS f_tj( X(uj)*c + Y(uj)*s , X(uj)*(-s) + Y(uj)*c );
    
//    VECTOR_CLASS f_ti(c, -s);
//    VECTOR_CLASS f_tj(s, c);


    EXPECT_TRUE( are_equal(tz, fc.to_basis(z)) );
    EXPECT_TRUE( are_equal(ti, fc.to_basis(i)) );
    EXPECT_TRUE( are_equal(tj, fc.to_basis(j)) );
    EXPECT_TRUE( are_equal(fz, fc.from_basis(z)) );
    EXPECT_TRUE( are_equal(fi, fc.from_basis(i)) );
    EXPECT_TRUE( are_equal(fj, fc.from_basis(j)) );
    EXPECT_TRUE( are_equal(f_tz, fc.to_frame(z)) );
    EXPECT_TRUE( are_equal(f_ti, fc.to_frame(i)) );
    EXPECT_TRUE( are_equal(f_tj, fc.to_frame(j)) );
    EXPECT_TRUE( are_equal(f_fz, fc.from_frame(z)) );
    EXPECT_TRUE( are_equal(f_fi, fc.from_frame(i)) );
    EXPECT_TRUE( are_equal(f_fj, fc.from_frame(j)) );
}

TEST(test_frame_changement, all){
    check_all<Vector2d>();
    check_all<rhoban_geometry::Point>();
    check_all<Eigen::Vector2d>();
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
