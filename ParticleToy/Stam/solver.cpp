//
// Created by appie on 13-6-2017.
//

#include <stdlib.h>
#include <Eigen/StdVector>
#include "solver.h"

#include "../Particle.h"


#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

void add_source ( int N, float * x, float * s, float dt )
{
    int i, size=(N+2)*(N+2);
    for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void set_bnd ( int N, int b, float * x, bool * boundary )
{
    int i,j;

    for ( i=1 ; i<=N ; i++ ) {
        x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
        x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
        x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
        x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
    }

    FOR_EACH_CELL
        if (boundary[IX(i,j)]) {
            x[IX(i-1,j)] = 0; //Is not allowed to come here
            x[IX(i-1,j)] = b==1 ? -x[IX(i-1,j)] : x[IX(i-1,j)];
            x[IX(i+1,j)] = b==2 ? -x[IX(i+1,1)] : x[IX(i+1,1)];
            x[IX(i,j-1)] = b==2 ? -x[IX(i,j-1)] : x[IX(i,j-1)];
            x[IX(i,j+1)] = b==2 ? -x[IX(i,j+1)] : x[IX(i,j+1)];
        }
    END_FOR

    x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
    x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
    x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
    x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
}

void compute_solid_force ( std::vector<Particle *> markers, std::vector<Particle *> solids ) {
    float kint = 0.5;
    float dint = 0.5;
    float zint = 0.5;
    Vec2f R = Vec2f(0.2, 0.2);

    for (auto& solid: solids){
        Vec2f fElem = Vec2f(0, 0);
        for (auto& marker : markers) {
//            double xDiff = sqrt( pow(marker->m_Position[0] - solid->m_Position[0], 2) + pow(marker->m_Position[1] - solid->m_Position[1], 2) );
//            printf("Distance x  %d\n", xDiff);
            Vec2f xDiff = marker->m_Position - solid->m_Position;
            if (xDiff[0] <= 1.0 || xDiff[1] <= 1.0){
                Vec2f distance = marker->m_Position - solid->m_Position;
                fElem = (-1 * kint) * ( distance - Vec2f(dint, dint) )
                              - (Vec2f(zint, zint) * ( marker->m_Velocity * distance ) );
                printf("Force %f %f \n", fElem[0], fElem[1]);
            }
        }
        solid->m_Force += fElem;
    }
}

void lin_solve ( int N, int b, float * x, float * x0, float a, float c, bool* boundary )
{
    int i, j, k;

    for ( k=0 ; k<20 ; k++ ) {
        FOR_EACH_CELL
                x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
        END_FOR
        set_bnd ( N, b, x, boundary );
    }
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt, bool* boundary )
{
    float a=dt*diff*N*N;
    lin_solve ( N, b, x, x0, a, 1+4*a, boundary );
}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt, bool* boundary )
{
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;

    dt0 = dt*N;
    FOR_EACH_CELL
            x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
            if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
            if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
            s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
            d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
                         s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
    END_FOR
    set_bnd ( N, b, d, boundary );
}

void project ( int N, float * u, float * v, float * p, float * div, bool* boundary )
{
    int i, j;

    FOR_EACH_CELL
            div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
            p[IX(i,j)] = 0;
    END_FOR
    set_bnd ( N, 0, div, boundary ); set_bnd ( N, 0, p, boundary );

    lin_solve ( N, 0, p, div, 1, 4, boundary );

    FOR_EACH_CELL
            u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
            v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
    END_FOR
    set_bnd ( N, 1, u, boundary ); set_bnd ( N, 2, v, boundary );
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt, bool* boundary )
{
    add_source ( N, x, x0, dt );
    SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt, boundary );
    SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, dt, boundary );
}

void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, bool* boundary )
{
    add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt );
    SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, dt, boundary );
    SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, dt, boundary );
    project ( N, u, v, u0, v0, boundary );
    SWAP ( u0, u ); SWAP ( v0, v );
    advect ( N, 1, u, u0, u0, v0, dt, boundary ); advect ( N, 2, v, v0, u0, v0, dt, boundary );
    project ( N, u, v, u0, v0, boundary );
}

