//
// Created by appie on 13-6-2017.
//

#include <Eigen/StdVector>
#include "../Macros.h"
#include "../particles/ParticleSystem.h"

void add_source(int N, float *x, float *s, float dt) {
    int i, size = (N + 2) * (N + 2);
    for (i = 0; i < size; i++) x[i] += dt * s[i];
}

void set_bnd(int N, int b, float *x, ParticleSystem &p) {
    int i, j;

    for (i = 1; i <= N; i++) {
        x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];
        x[IX(N + 1, i)] = b == 1 ? -x[IX(N, i)] : x[IX(N, i)];
        x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
        x[IX(i, N + 1)] = b == 2 ? -x[IX(i, N)] : x[IX(i, N)];
    }

    for (auto &rigidParticle : p.rigidParticles) {
        for (auto &polyCell : rigidParticle->polyEdgeCells) {
            i = polyCell[0];
            j = polyCell[1];
            if (i > 0 && i < N && j > 0 && j <= N) {
                x[IX(i - 1, j)] = 0; //Is not allowed to come here
                x[IX(i - 1, j)] = b == 1 ? -x[IX(i - 1, j)] : x[IX(i - 1, j)];
                x[IX(i + 1, j)] = b == 2 ? -x[IX(i + 1, 1)] : x[IX(i + 1, 1)];
                x[IX(i, j - 1)] = b == 2 ? -x[IX(i, j - 1)] : x[IX(i, j - 1)];
                x[IX(i, j + 1)] = b == 2 ? -x[IX(i, j + 1)] : x[IX(i, j + 1)];
            }
        }
    }
    x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
    x[IX(0, N + 1)] = 0.5f * (x[IX(1, N + 1)] + x[IX(0, N)]);
    x[IX(N + 1, 0)] = 0.5f * (x[IX(N, 0)] + x[IX(N + 1, 1)]);
    x[IX(N + 1, N + 1)] = 0.5f * (x[IX(N, N + 1)] + x[IX(N + 1, N)]);
}

void lin_solve(int N, int b, float *x, float *x0, float a, float c, ParticleSystem &p) {
    int i, j, k;

    for (k = 0; k < 20; k++) {
        FOR_EACH_CELL
                x[IX(i, j)] =
                        (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) /
                        c;
        END_FOR
        set_bnd(N, b, x, p);
    }
}

float vorticity_vector(int i, int j, float *u, float *v) {
    int N = 64;
    return u[IX(i + 1, j)] - u[IX(i, j)] + v[IX(i, j + 1)] - v[IX(i, j)];
}

/**
 * Implements vorticity confinement
 */
void vorticity_confinement(float *u, float *v, int N, float dt) {
    int i, j;
    float epsilon = 0.1f;

    FOR_EACH_CELL
            if (i != 1 && j != 1 && i != N && j != N) {
                // gradient * u
                float du = vorticity_vector(i + 1, j, u, v) - vorticity_vector(i, j, u, v);
                float dv = vorticity_vector(i, j + 1, u, v) - vorticity_vector(i, j, u, v);

                float length = sqrtf(du * du + dv * dv);

                float frac = length / 2;

                u[IX(i, j)] += epsilon * dt * (frac * du) * 5;
                v[IX(i, j)] += epsilon * dt * (frac * dv) * 5;
            }
    END_FOR
}

void diffuse(int N, int b, float *x, float *x0, float diff, float dt, ParticleSystem &p) {
    float a = dt * diff * N * N;
    lin_solve(N, b, x, x0, a, 1 + 4 * a, p);
}

void advect(int N, int b, float *d, float *d0, float *u, float *v, float dt, ParticleSystem &p) {
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;

    dt0 = dt * N;
    FOR_EACH_CELL
            x = i - dt0 * u[IX(i, j)];
            y = j - dt0 * v[IX(i, j)];
            if (x < 0.5f) x = 0.5f;
            if (x > N + 0.5f) x = N + 0.5f;
            i0 = (int) x;
            i1 = i0 + 1;
            if (y < 0.5f) y = 0.5f;
            if (y > N + 0.5f) y = N + 0.5f;
            j0 = (int) y;
            j1 = j0 + 1;
            s1 = x - i0;
            s0 = 1 - s1;
            t1 = y - j0;
            t0 = 1 - t1;
            d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) +
                          s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);
    END_FOR
    set_bnd(N, b, d, p);
}

void project(int N, float *u, float *v, float *p, float *div, ParticleSystem &particleSystem) {
    int i, j;

    FOR_EACH_CELL
            div[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / N;
            p[IX(i, j)] = 0;
    END_FOR
    set_bnd(N, 0, div, particleSystem);
    set_bnd(N, 0, p, particleSystem);

    lin_solve(N, 0, p, div, 1, 4, particleSystem);

    FOR_EACH_CELL
            u[IX(i, j)] -= 0.5f * N * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
            v[IX(i, j)] -= 0.5f * N * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
    END_FOR
    set_bnd(N, 1, u, particleSystem);
    set_bnd(N, 2, v, particleSystem);
}

void dens_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt, ParticleSystem &particleSystem) {
    add_source(N, x, x0, dt);
    SWAP (x0, x);
    diffuse(N, 0, x, x0, diff, dt, particleSystem);
    SWAP (x0, x);
    advect(N, 0, x, x0, u, v, dt, particleSystem);
}

void vel_step(int N, float *u, float *v, float *u0, float *v0, float visc, float dt, ParticleSystem &particleSystem) {
    add_source(N, u, u0, dt);
    add_source(N, v, v0, dt);
    SWAP (u0, u);
    diffuse(N, 1, u, u0, visc, dt, particleSystem);
    SWAP (v0, v);
    diffuse(N, 2, v, v0, visc, dt, particleSystem);
    project(N, u, v, u0, v0, particleSystem);
    SWAP (u0, u);
    SWAP (v0, v);
    advect(N, 1, u, u0, u0, v0, dt, particleSystem);
    advect(N, 2, v, v0, u0, v0, dt, particleSystem);
    project(N, u, v, u0, v0, particleSystem);
}