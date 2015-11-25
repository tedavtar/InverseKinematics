#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;

// CS184 Simple OpenGL Example
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif


float lambda = 0.005f;

float threshold = 0.1f;

float chopfactor = 0.1f;

Vector3f r1best;
Vector3f r2best;
Vector3f r3best;
Vector3f r4best;

float bestDist=100;


float maxTrys = 500; // if after maxTrys times, we just give up on that goal

//our 4 end effector points and origin (in world space coordinates)

Vector3f ori(0.0f,0.0f,0.0f);
Vector3f p1(.375f,0.0f,0.0f);
Vector3f p2(.625f,0.0f,0.0f);
Vector3f p3(.75f,0.0f,0.0f);
Vector3f p4(.875f,0.0f,0.0f);


Vector3f goal(1.0f, 1.0f, 0.0f);



/*
//segment rotations
Vector3f s1rot(0.0f,0.0f,0.0f);
Vector3f s2rot(0.0f,0.0f,0.0f);
Vector3f s3rot(0.0f,0.0f,0.0f);
Vector3f s4rot(0.0f,0.0f,0.0f);*/

//http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257 got code from here

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

//some math helpers

//returns the cross product matrix [x].  (so x cross v = [x]v)
Matrix3f crossProdMatrix(Vector3f x){
    Matrix3f rtn;
    
    rtn << 0    , -x(2), x(1) ,
           x(2) , 0    , -x(0),
           -x(1), x(0) , 0    ;
    
    return rtn;
}

//uses RR on ri to get Ri
Matrix3f buildRMatrix(Vector3f r){
    
    
    
    Matrix3f rtn;
    float theta = r.norm();
    
    if (theta < .0000001) {
        //so since theta so small, it's like we're not even rotating -- so return Identity matrix?
        
        //theta = .0000001; //to prevent divide by 0 errors NOT WORKING
        
        Matrix3f i3;
        i3<<  1, 0, 0,
        0, 1, 0,
        0, 0, 1;
        return i3;
    }
    
    //cout << "theta: " << theta << endl; //correct-got the magnitude
    //cout << r/theta << endl; //ok
    float costheta = cosf(theta);
    float sintheta = sinf(theta);
    
    //normalize r
    Vector3f rhat = r/theta;
    
    Matrix3f rhatcross = crossProdMatrix(rhat);
    Matrix<float, 1, 3> rhattransposed = rhat.transpose();
    
    rtn = rhat*rhattransposed + sintheta*rhatcross - costheta*rhatcross*rhatcross;
    
    return rtn;
}





class BallJoint {
public:
    float length;
    
    Vector3f rVector; //this is ri -- the exponential map vector.
    
    Matrix3f rMatrix(){//this is Ri -- which incidentally (I believe) is an orthonormal basis for joint i+1 expressed in this joint (i's) local coordinate system
        
        return buildRMatrix(rVector);
    };
    
    Vector3f endPoint(){//this is pi
        Vector3f start;
        start << length, 0.0f, 0.0f;
        
        //cout << rMatrix() << endl;
        
        return rMatrix()*start;
    }
    
    Vector4f endPointH(){//this is pi
        Vector4f rtn(0,0,0,1);
        
        rtn.block<3,1>(0,0) = endPoint();

        
        return rtn;
    }
    
    Matrix4f xMatrix(){
        Matrix4f rtn;
        
        //populates top left block with Ri
        rtn.block<3,3>(0,0) = rMatrix();
        
        //populates right colum with pi
        rtn.block<3,1>(0,3) = endPoint();
        
        //populates bottom row with the (0,0,0,1)
        Vector4f btmRow(0.0f,0.0f,0.0f,1.0f);
        rtn.block<1,4>(3,0) = btmRow;
        
        
        
        return rtn;
    }
    
    
    int ID; //can be 1 2 3 4 (only use so far thought of is for coloring)
    
    BallJoint(float Tlength,  Vector3f TrVector  ,int Tid):length(Tlength),ID(Tid), rVector(TrVector){}
    
    //Segment(){}? args?
   
};


vector<BallJoint> segments;

void initializeSegments(){
    //create 4 ball joints, load them to segments, and set rotations to 0
    BallJoint b1 = BallJoint(.375f, Vector3f(0.0f,0.0f,0.0f), 1);
    BallJoint b2 = BallJoint(.25f, Vector3f(0.0f,0.0f,0.0f), 2);
    BallJoint b3 = BallJoint(.125f, Vector3f(0.0f,0.0f,0.0f), 3);
    BallJoint b4 = BallJoint(.125f, Vector3f(0.0f,0.0f,0.0f), 4);
    segments.push_back(b1);
    segments.push_back(b2);
    segments.push_back(b3);
    segments.push_back(b4);
    
}


//this takes a two points and draws a line between them with segment color
void drawSegment(Vector3f p1, Vector3f p2, int segment)//so based on segment we color differently
{
    glLineWidth(5.0f);
    
    glBegin(GL_LINES);
    /*1st segment Red
     *2nd Green
     *3rd Blue
     *4th Purple
     */
    switch (segment) {
        case 1:
            glColor3f(1.0f,0.0f,0.0f);
            break;
        case 2:
            glColor3f(0.0f,1.0f,0.0f);
            break;
        case 3:
            glColor3f(0.0f,0.0f,1.0f);
            break;
        case 4:
            glColor3f(1.0f,0.0f,1.0f);
            break;
    }
    
    glVertex3f(p1(0),p1(1),p1(2));
    glVertex3f(p2(0),p2(1),p2(2));
    
    
    glEnd();
    
}


void drawSegments(){
    maxTrys -= 1;
    //compute dist from goal
    Vector3f disp = p4 - goal;
    float dist = disp.norm();

    Vector3f oldP1 = segments[0].rVector;
    Vector3f oldP2 = segments[1].rVector;
    Vector3f oldP3 = segments[2].rVector;
    Vector3f oldP4 = segments[3].rVector;
    
    if (dist < bestDist) {
        bestDist = dist;
        r1best = oldP1;
        r2best = oldP2;
        r3best = oldP3;
        r4best = oldP4;
    }
    
    //cout << dist << endl;
    
    if (dist < threshold || maxTrys < 0) {
       
        //so since reached here, we're calling it quits on current goal. Might as well show the best/closest we came
        
        segments[0].rVector = r1best;
        segments[1].rVector = r2best;
        segments[2].rVector = r3best;
        segments[3].rVector = r4best;

        
        
        //now use the ri (local) to get pi in world (so replace p1,p2,p3,p4)
        Matrix4f f4 = segments[0].xMatrix()*segments[1].xMatrix()*segments[2].xMatrix(); //X4to1
        Matrix4f f3 = segments[0].xMatrix()*segments[1].xMatrix(); //X3to1
        Matrix4f f2 = segments[0].xMatrix(); //X2to1
        
        //cout << f2 << endl;
        //and clearly matrix X1 to X1 is just identity
        
        Vector4f h4 = f4*segments[3].endPointH();
        Vector4f h3 = f3*segments[2].endPointH();
        Vector4f h2 = f2*segments[1].endPointH();
        
        p4.block<3,1>(0,0) = h4.block<3,1>(0,0);
        p3.block<3,1>(0,0) = h3.block<3,1>(0,0);
        p2.block<3,1>(0,0) = h2.block<3,1>(0,0);
        p1 = segments[0].endPoint();

         //print out final position
        cout << p4 << endl;
        
        
        
        
        lambda = 0.01;
        //draw segment 1
        drawSegment(ori,p1,1);
        
        //draw segment 2
        drawSegment(p1,p2,2);
        
        //draw segment 3
        drawSegment(p2,p3,3);
        
        //draw segment 4
        drawSegment(p3,p4,4);
        
        //cout << p4 << endl;
        return; //REPLACE THIS BY CHANGE GOAL TO NEXT GOAL POSITION AND THEN RETURN
    }
    

    
    //compute Jis and then concatenate them to get 3 by 12 final Jacobian
    Matrix3f J1, J2, J3, J4;
    
    //start off getting pn (p4 but locally-so in bj4 coords)
    Vector3f pn = segments[3].endPoint();
    //cout << pn << endl;
    
    Vector4f homoPN(0,0,0,1);
    homoPN.block<3,1>(0,0) = pn;
    
    
    
    //compute J1:
    //so no need to find R1->0 as no need to rotate because already in world coordinates
    //so X4->1 = X1*X2*X3 as need to transform pn from bj4 coords to bj1 coords
    
    Matrix4f X4to1 = segments[0].xMatrix()*segments[1].xMatrix()*segments[2].xMatrix();
    
    Vector4f homoPN1 = X4to1*homoPN;
    
    Vector3f result1(0,0,0);
    result1.block<3,1>(0,0) = homoPN1.block<3,1>(0,0);
    
    //cout << result1 << endl;
    
     J1 = -1 * crossProdMatrix(result1);
     //cout << J1 << endl;
    
    
    
    /*
    J1 = -1 * crossProdMatrix(p4);
    cout << J1 << endl;
     */
    
    
    
    //compute J2
    //so need to find R2->0 or rotate from bj2 to bj1--which is just R1
    Matrix3f R2to0 = segments[0].rMatrix();//R1
    
    Matrix4f X4to2 = segments[1].xMatrix()*segments[2].xMatrix();
    Vector4f homoPN2 = X4to2*homoPN;
    Vector3f result2(0,0,0);
    result2.block<3,1>(0,0) = homoPN2.block<3,1>(0,0);
    
    J2 = -1 * R2to0 * crossProdMatrix(result2);
    //cout << J2 << endl;
    
    
    //compute J3
    //so need to find R3->0 or rotate from bj3 to bj1--so 1st multiply by R2 to go to bj2 then by R1
    Matrix3f R3to0 = segments[0].rMatrix()*segments[1].rMatrix();//R1*R2
    //so X4->3 = X3
    Matrix4f X4to3 = segments[2].xMatrix();
    Vector4f homoPN3 = X4to3*homoPN;
    Vector3f result3(0,0,0);
    result3.block<3,1>(0,0) = homoPN3.block<3,1>(0,0);
    
    J3 = -1 * R3to0 * crossProdMatrix(result3);
    //cout << J3 << endl;
    
    //compute J4
    Matrix3f R4to0 = segments[0].rMatrix()*segments[1].rMatrix()*segments[2].rMatrix();//R1*R2*R3
    //now no need for translation, since pn already in bj4 coord system
    
    J4 = -1 * R4to0 * crossProdMatrix(pn);
    //cout << J4 << endl;
    
    
    
    Matrix<float, 3, 12> Jacobian;
    Jacobian << J1, J2, J3, J4;
    
    //cout << Jacobian << endl;
    MatrixXf jac(3,12);
    for (int i=0; i<3; i++) {
        for (int j=0; j<12; j++) {
            jac(i,j) = Jacobian(i,j);
        }
    }
    
    Matrix<float, 12, 3> JInv = pseudoInverse(jac);
    
    //cout << JInv << endl;
    
    //so we already have pe. which in addition to being p4 (world) is also "result1"
    Matrix<float, 12, 1> dr = JInv * lambda * (goal - result1);
    //cout << dr << endl;
    
    //now update all segment's rotations from the dr
    Vector3f dr1 = dr.block<3,1>(0,0);
    Vector3f dr2 = dr.block<3,1>(3,0);
    Vector3f dr3 = dr.block<3,1>(6,0);
    Vector3f dr4 = dr.block<3,1>(9,0);
    
    segments[0].rVector += dr1;
    segments[1].rVector += dr2;
    segments[2].rVector += dr3;
    segments[3].rVector += dr4;
    
    //now use the ri (local) to get pi in world (so replace p1,p2,p3,p4)
    Matrix4f f4 = segments[0].xMatrix()*segments[1].xMatrix()*segments[2].xMatrix(); //X4to1
    Matrix4f f3 = segments[0].xMatrix()*segments[1].xMatrix(); //X3to1
    Matrix4f f2 = segments[0].xMatrix(); //X2to1
    
    //cout << f2 << endl;
    //and clearly matrix X1 to X1 is just identity
    
    Vector4f h4 = f4*segments[3].endPointH();
    Vector4f h3 = f3*segments[2].endPointH();
    Vector4f h2 = f2*segments[1].endPointH();
    
    p4.block<3,1>(0,0) = h4.block<3,1>(0,0);
    p3.block<3,1>(0,0) = h3.block<3,1>(0,0);
    p2.block<3,1>(0,0) = h2.block<3,1>(0,0);
    p1 = segments[0].endPoint();
    
    
    //cout<< p2 << endl;
    
    Vector3f newdisp = p4 - goal;
    float newdist = newdisp.norm();
    
    
    
    if (newdist > dist) {
        segments[0].rVector = oldP1;
        segments[1].rVector = oldP2;
        segments[2].rVector = oldP3;
        segments[3].rVector = oldP4;
        //lambda *= .5;
        
        segments[0].rVector += dr1*chopfactor;
        segments[1].rVector += dr2*chopfactor;
        segments[2].rVector += dr3*chopfactor;
        segments[3].rVector += dr4*chopfactor;
        return;
    }
    

    
     //draw segment 1
     drawSegment(ori,p1,1);
     
     //draw segment 2
     drawSegment(p1,p2,2);
     
     //draw segment 3
     drawSegment(p2,p3,3);
     
     //draw segment 4
     drawSegment(p3,p4,4);
    


}


void display(void)//display the 4 segments
{
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    drawSegments();
    
    
    
    glFlush ();
}

void reshape (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    if (w <= h)
        glOrtho (-1.5, 1.5, -1.5*(GLfloat)h/(GLfloat)w,
                 1.5*(GLfloat)h/(GLfloat)w, -10.0, 10.0);
    else
        glOrtho (-1.5*(GLfloat)w/(GLfloat)h,
                 1.5*(GLfloat)w/(GLfloat)h, -1.5, 1.5, -10.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void refresh(){
#ifndef _WIN32
    //Sleep(10);
#endif
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    //the usual sanity tests hastily thrown in main mostly to make sure I'm using Eigen correctly
    /*
    Vector3f pt(1.0f,2.0f,3.0f);
    cout << pt(1) << endl;*/ //how to index vector -- 0 indexed
    /*
    Vector3f p(10.0f,0.0f,0.0f);
    Matrix3f t = crossProdMatrix(p);
    cout << t << endl;*/
    /*
    float tangle = 3.14f;  //just testing to make sure trig is in radians--because the taylor series stuff relies on this
    float cos = sinf(tangle);
    cout << cos << endl;
    */
    
    Vector3f p(3.0f,4.0f,5.0f);
    /*
    Matrix<float, 1, 3> ptransposed = p.transpose();
    cout << "p:\n" << p << "\np:transposed: " << ptransposed << endl;*/
    /* verify operations (and their order)
    Matrix2f t;
    t<< 2, 0, 0, 2;
    cout << t*t + .5*t << endl;
     */
    
    /* //BLOCK MATRICES STITCH TOGETHER EXAMPLE!
    Matrix2f t;
    t<< 2, 2, 2, 2;
    Vector3f btmRow(0.0f,0.0f,1.0f);
    Vector2f rightCol(0.0f,3.0f);
    //Matrix<float, 1, 3> p2 = p1.transpose();
    Matrix3f combined;
    combined.block<2,2>(0,0) = t;
    combined.block<2,1>(0,2) = rightCol;
    combined.block<1,3>(2,0) = btmRow;
    cout << combined << endl;
    */
    
    //buildRMatrix(p);
    
    initializeSegments();
    
    
    /*
    MatrixXd testPinv(3,12);
    testPinv << 0,  0,    0,  0,  0,  0, 0,   0,  0,  0,  0,  0,
    0,  0,   10,  0,  0,  9, 0,   0,  7,  0,  0,  4,
    0,-10 ,   0,  0, -9,  0, 0,  -7,  0,  0, -4,  0;
    cout << pseudoInverse(testPinv) << endl;
    */
    
    
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize (500, 500);
    glutInitWindowPosition (100, 100);
    glutCreateWindow (argv[0]);
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(refresh);
    glutMainLoop();
    return 0;
}


/*#include <iostream>
 #include <Eigen/Dense>
 
 using namespace Eigen;
 using namespace std;
 
 
 
 
 int main()
 {
 MatrixXd m(2,2);
 m(0,0) = 3;
 m(1,0) = 2.5;
 m(0,1) = -1;
 m(1,1) = m(1,0) + m(0,1);
 std::cout << "Here is the matrix m:\n" << m << std::endl;
 VectorXd v(2);
 v(0) = 4;
 v(1) = v(0) - 1;
 std::cout << "Here is the vector v:\n" << v << std::endl;
 }*/