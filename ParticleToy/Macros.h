//
// Created by Tomirio on 19-6-2017.
//

#ifndef SIMULATION_2_MACROS_H
#define SIMULATION_2_MACROS_H

#define IX(i, j) ((i)+(N+2)*(j))
#define SWAP(x0, x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

#endif //SIMULATION_2_MACROS_H
