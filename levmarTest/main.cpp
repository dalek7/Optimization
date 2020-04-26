#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// gcc -L/usr/local/lib  -L. lmdemo.o -o lmdemo -llevmar -llapack -lblas -lf2c  -lm


#include <levmar.h>

#ifndef LM_DBL_PREC
#error Example program assumes that levmar has been compiled with double precision, see LM_DBL_PREC!
#endif

/* the following macros concern the initialization of a random number generator for adding noise */
#undef REPEATABLE_RANDOM /* #define this for repeatable program behavior across runs */
#define DBL_RAND_MAX (double)(RAND_MAX)

#ifdef _MSC_VER // MSVC
#include <process.h>
#define GETPID  _getpid
#elif defined(__GNUC__) // GCC
#include <sys/types.h>
#include <unistd.h>
#define GETPID  getpid
#else
#warning Do not know the name of the function returning the process id for your OS/compiler combination
#define GETPID  0
#endif /* _MSC_VER */

#ifdef REPEATABLE_RANDOM
#define INIT_RANDOM(seed) srandom(seed)
#else
#define INIT_RANDOM(seed) srandom((int)GETPID()) // seed unused
#endif

using namespace std;

//using namespace DD;
/* Gaussian noise with mean m and variance s, uses the Box-Muller transformation */
double gNoise(double m, double s)
{
    double r1, r2, val;

    r1=((double)random())/DBL_RAND_MAX;
    r2=((double)random())/DBL_RAND_MAX;

    val=sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2);

    val=s*val+m;

    return val;
}


/* model to be fitted to measurements: x_i = p[0]*exp(-p[1]*i) + p[2], i=0...n-1 */
void expfunc(double *p, double *x, int m, int n, void *data)
{
    register int i;

    for(i=0; i<n; ++i)
    {
        x[i]=p[0]*exp(-p[1]*i) + p[2];
    }
}

/* Jacobian of expfunc() */
void jacexpfunc(double *p, double *jac, int m, int n, void *data)
{
    register int i, j;

    /* fill Jacobian row by row */
    for(i=j=0; i<n; ++i)
    {
        jac[j++]=exp(-p[1]*i);
        jac[j++]=-p[0]*i*exp(-p[1]*i);
        jac[j++]=1.0;
    }
}



int main()
{
    const int n=50, m=3; // 50 measurements, 3 parameters
    double p[m], x[n], opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    register int i;
    int ret;

    /* generate some measurement using the exponential model with
    * parameters (5.0, 0.1, 1.0), corrupted with zero-mean
    * Gaussian noise of s=0.1
    */
    printf("Samples with parameters (5.0, 0.1, 1.0)\n");

    INIT_RANDOM(0);
    for(i=0; i<n; ++i)
        x[i]=(5.0*exp(-0.1*i) + 1.0) + gNoise(0.0, 0.1);

    /* initial parameters estimate: (1.0, 0.0, 0.0) */
    p[0]=1.0;
    p[1]=0.0;
    p[2]=0.0;

    /* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */
    opts[0]=LM_INIT_MU;
    opts[1]=1E-15;
    opts[2]=1E-15;
    opts[3]=1E-20;
    opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used

    /* invoke the optimization function */
    ret     =dlevmar_der(expfunc, jacexpfunc, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian

    printf("\n");
    printf("With Jacobian\n");
    printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
    printf("Best fit parameters: %.7g\t%.7g\t%.7g\n", p[0], p[1], p[2]);

    FILE *fp = fopen("out/out_obs.txt", "w+");

    for (int i=0; i<n; i++)
    {
        fprintf(fp, "%d\t%f\n", i, x[i]);
    }
    fclose(fp);

    fp = fopen("out/out_params_est_w_analytic_jacobian.txt", "w+");
    fprintf(fp, "%f\t%f\t%f\n", p[0], p[1], p[2]);
    fclose(fp);

    double p2[m];
    p2[0]=1.0;
    p2[1]=0.0;
    p2[2]=0.0;
    printf("\n");
    printf("Without Jacobian\n");
    ret=dlevmar_dif(expfunc, p2, x, m, n, 1000, opts, info, NULL, NULL, NULL); // without Jacobian
    printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
    printf("Best fit parameters: %.7g\t%.7g\t%.7g\n", p2[0], p2[1], p2[2]);


    fp = fopen("out/out_params_est_wo_jacobian.txt", "w+");
    fprintf(fp, "%f\t%f\t%f\n", p2[0], p2[1], p2[2]);
    fclose(fp);


    return 0;
}
