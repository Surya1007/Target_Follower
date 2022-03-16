/*
 * KalmanFilter.h
 *
 *  Created on: 28-Feb-2022
 *      Author: surya
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_


#include <stdio.h>
#include <math.h>


// Matrix multiplication
void multiplyMatrices(int R1, int comm, int C2, float rslt[R1][C2], float mat1[R1][comm], float mat2[comm][C2])
{
	// Loop through rows of result matrix
	for(int i = 0; i < R1; i++)
	{
		// Loop through columns of result matrix
		for (int j = 0; j < comm; j++)
		{
			// Loop for finding an element of result matrix
			for (int k = 0; k < C2; k++)
			{
				// Multiplying each element of a and b and storing it in c matrix
				rslt[i][j] =  rslt[i][j] + ((mat1[i][k]) * (mat2[k][j]));
			}
		}
	}
}





// Matrix Addition
void addition_mat(int R, int C, float c[R][C], float a[R][C], float b[R][C])
{
	// Loop through rows
	for(int i = 0; i < R; i++)
	{
		// Loop through columns
		for(int j = 0 ; j < C; j++)
		{
			// Adding each element of a and b and storing it in c matrix
			c[i][j] = a[i][j] + b[i][j];
		}
	}
}




// Matrix subtraction
void subtraction_mat(int R, int C, float c[R][C], float a[R][C], float b[R][C])
{
	// Loop through rows
	for(int i = 0; i < R; i++)
	{
		// Loop through columns
		for(int j = 0 ; j < C; j++)
		{
			// Subtracting each element of a from b and storing it in c matrix
			c[i][j] = a[i][j] - b[i][j];
		}
	}
}





// Finding transpose of matrix
void transpose_matrix(int R, int C, float transpose[R][C], float mat[R][C])
{
	/*
	 * Function to find the transpose of a matrix
	 *
	 * Arguments:
	 *
	 * Outputs:
	 *
	 * Inputs:
	 * mat: Input matrix
	 * R: number of rows in input matrix
	 * C: number of columns in input matrix
	 *
	 * */
	for(int i = 0; i < R; i++)
	{
		for(int j = 0; j < C; j++)
		{
			transpose[j][i] = mat[i][j];
		}
	}
}



/*For calculating Determinant of the Matrix */
float determinant(int k, float a[k][k])
{
	/*
	 * Arguments:
	 *
	 *
	 * Inputs:
	 *
	 *
	 *
	 */
	float s = 1, det = 0, b[k][k];
	int i, j, m, n, c;
	if (k == 1)
	{
		return (a[0][0]);
	}
	else
	{
		det = 0;
		for (c = 0; c < k; c++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < k; i++)
			{
				for (j = 0 ; j < k; j++)
				{
					b[i][j] = 0;
					if (i != 0 && j != c)
					{
						b[m][n] = a[i][j];
						if (n < (k - 2))
							n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s * (a[0][c] * determinant(k - 1, b));
			s = -1 * s;
		}
	}

	return det;
}

void cofactor(int f, float num[f][f], float inverse[f][f])
{
	float b[f][f], fac[f][f];
	int p, q, m, n, i, j;
	for (q = 0; q < f; q++)
	{
		for (p = 0; p < f; p++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < f; i++)
			{
				for (j = 0; j < f; j++)
				{
					if (i != q && j != p)
					{
						b[m][n] = num[i][j];
						if (n < (f - 2))
							n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			fac[q][p] = pow(-1, q + p) * determinant(f - 1, b);
		}
	}

	float d = determinant(f, num);
	for (i = 0; i < f; i++)
	{
		for (j = 0; j < f; j++)
		{
			inverse[i][j] = fac[j][i] / d;
		}
	}
}

void inverse_matrix(int R, float inversed[R][R], float a[R][R])
{
	float d = determinant(R, a);
	if (d == 0)
		printf("\nInverse of Matrix is not possible\n");
	else
		cofactor(R, a, inversed);
}

void kalman_filter(float X_k[6][1], float U_k[3][1], float delta_t, float Z_k[7][1])
{
	// State estimate of system of size 6 x 1
	//static float X_k [6][1] = {{0}, {0}, {0}, {0}, {0}, {0}};

	// Matrix F of size 6 x 6
	float F[6][6] = {{1, 0, 0, delta_t, 	0, 		0},
			   {0, 1, 0, 	0, 	delta_t, 	0},
			   {0, 0, 1, 	0, 		0, 	delta_t},
			   {0, 0, 0, 	1, 		0, 		0},
			   {0, 0, 0, 	0, 		1, 		0},
			   {0, 0, 0, 	0, 		0, 		1}};

	// Matrix G of size 6 x 3
	float G[6][3] = {	{delta_t, 	0, 		0},
				{0, 	delta_t, 	0},
				{0, 		0, delta_t},
				{1, 		0, 		0},
				{0, 		1, 		0},
				{0, 		0, 		1}};

	// Matrix P of size 6 x 6
	static float P_k[6][6] = {	{500, 0, 0, 500, 0, 0},
							{0, 500, 0, 0, 500, 0},
							{0, 0, 500, 0, 0, 500},
							{500, 0, 0, 500, 0, 0},
							{0, 500, 0, 0, 500, 0},
							{0, 0, 500, 0, 0, 500}};

	// Matrix Q of size 6 x 6
	float sig_ax = 0.1, sig_ay = 0.1, sig_alpha = 0.1;

	float Q[6][6] = {	{pow(delta_t, 4) * pow(sig_ax, 2) / 4,		0,	0,	pow(delta_t, 3) * pow(sig_ax, 2) / 2,	0,	0},
				{0,	pow(delta_t, 4) * pow(sig_ay, 2) / 4,	0,	0, 	pow(delta_t, 3) * pow(sig_ay, 2) / 2,	0},
				{0,	0,	pow(delta_t, 4) * pow(sig_alpha, 2) / 4,	0,	0,	pow(delta_t, 3) * pow(sig_alpha, 2) / 2},
				{pow(delta_t, 3) * pow(sig_ax, 2) / 2,		0,	0,	pow(delta_t, 2) * pow(sig_ax, 2),	0,	0},
				{0,	pow(delta_t, 3) * pow(sig_ay, 2) / 2,	0,	0, 	pow(delta_t, 2) * pow(sig_ay, 2),	0},
				{0,	0,	pow(delta_t, 3) * pow(sig_alpha, 2) / 2,	0,	0,	pow(delta_t, 2) * pow(sig_alpha, 2)}};

	// Matrix H of size 7 x 6
	float H[7][6] = {	{1 / pow(delta_t, 2), 0, 0, 1 / delta_t, 0, 0},
				{0, 1 / pow(delta_t, 2), 0, 0, 1 / delta_t, 0},
				{0, 0, 1, 0, 0, delta_t},
				{0, 0, 1 / delta_t, 0, 0, 1},
				{1 / delta_t, 0, 0, 1, 0, 0},
				{0, 1 / delta_t, 0, 0, 1, 0},
				{0, 0, 1 / delta_t, 0, 0, 1}};

	// Matrix R of size 7 x 7
	float R_k[7][7] = {	{0.1, 0, 0, 0, 0, 0, 0},
				{0, 0.1, 0, 0, 0, 0, 0},
				{0, 0, 0.1, 0.1, 0, 0, 0},
				{0, 0, 0.1, 0.1, 0, 0, 0},
				{0, 0, 0, 0, 0.1, 0, 0},
				{0, 0, 0, 0, 0, 0.1, 0},
				{0, 0, 0, 0, 0, 0, 0.1}};

	// Prediction stage
	// X_k_prio = F * X_(k-1) + G * U_(k-1)
	float X_k_prio[6][1] = {{0}, {0}, {0}, {0}, {0}, {0}}, P_k_prio[6][6], P_k_prio_half[6][6], X_k_prio_half[6][1];
	multiplyMatrices(6, 6, 1, X_k_prio, F, X_k);
	printf("Matrix is : \n ");
	for (uint8_t i = 0; i < 6; i++)
	{
		for (uint8_t j = 0; j < 1; j++)
		{
			printf("%f ", X_k[i][j]);
		}
		printf("\n");
	}
	multiplyMatrices(6, 3, 1, X_k_prio_half, G, U_k);
	addition_mat(6, 1, X_k_prio, X_k_prio, X_k_prio_half); ////////////

	float F_T[6][6];
	// P_k_prio = F * P_k * FT + Q
	multiplyMatrices(6, 6, 6, P_k_prio_half, F, P_k);
	// To find F transpose
	transpose_matrix(6, 6, F_T, F);
	multiplyMatrices(6, 6, 6, P_k_prio, P_k_prio_half, F_T);
	addition_mat(6, 6, P_k_prio, P_k_prio, Q); ///////////////////////////////




	// Correction stage
	// kalman_gain = (P_k_prio*H.T) / (H*P_k_prio*H.T + R_k)
	float kalman_gain[6][7];
	float H_T[6][7], kf_num[6][7], kf_den[6][7], kf_den_half[6][7], kf_den_half1[6][7];
	// Numerator calculation of kalman gain
	transpose_matrix(7, 6, H_T, H);
	multiplyMatrices(6, 6, 7, kf_num, P_k_prio, H_T);

	// Denominator calculation of kalman gain
	multiplyMatrices(7, 6, 6, kf_den_half1, H, P_k_prio);
	multiplyMatrices(7, 6, 7, kf_den_half, kf_den_half1, H_T);
	addition_mat(7, 7, kf_den_half1, kf_den_half, R_k);
	// Find inverse of kf_den and multiply with kf_num to get kalman_gain



	inverse_matrix(7, kf_den, kf_den_half1);

	multiplyMatrices(6, 7, 7, kalman_gain, kf_num, kf_den);






	// X_k = X_k_prio + kalman_gain * (Z_k - (H * X_k_prio))
	float X_k_half[6][1], X_k_half1[6][1];
	multiplyMatrices(7, 6, 1, X_k_half, H, X_k_prio);
	subtraction_mat(7, 1, X_k_half1, Z_k, X_k_half);
	multiplyMatrices(6, 7, 1, X_k_half,kalman_gain, X_k_half1);
	addition_mat(6, 1, X_k, X_k_prio, X_k_half);
	printf("Matrix is : \n ");
	for (uint8_t i = 0; i < 6; i++)
	{
		for (uint8_t j = 0; j < 1; j++)
		{
			printf("%f ", X_k[i][j]);
		}
		printf("\n");
	}

	// P_k = ((I - kalman_gain * H) * P_k_prio * (I - kalman_gain * H).T) + (kalman_gain * R_k * kalman_gain.T)
	float dummy[6][6], dummy_tran[6][6];
	float I[6][6] = {	{1, 0, 0, 0, 0, 0},
				{0, 1, 0, 0, 0, 0},
				{0, 0, 1, 0, 0, 0},
				{0, 0, 0, 1, 0, 0},
				{0, 0, 0, 0, 1, 0},
				{0, 0, 0, 0, 0, 1}};

	multiplyMatrices(6, 7, 6, dummy, kalman_gain, H);
	subtraction_mat(6, 6, dummy,I, dummy); ///////////////////////
	transpose_matrix(6, 6, dummy_tran, dummy);

	float P_k1[6][6], P_k2[6][6];
	multiplyMatrices(6, 6, 6, P_k1, dummy, P_k_prio);
	multiplyMatrices(6, 6, 6, P_k2, P_k1, dummy_tran);

	float dummy2[6][7], kalman_gain_T[7][6];
	multiplyMatrices(6, 7, 7, dummy2, kalman_gain, R_k);
	transpose_matrix(6, 7, kalman_gain_T, kalman_gain);
	multiplyMatrices(6, 7, 6, P_k1, dummy2, kalman_gain_T);

	addition_mat(6, 6, P_k, P_k2, P_k1);

	printf("Matrix is : \n ");
	for (uint8_t i = 0; i < 6; i++)
	{
		for (uint8_t j = 0; j < 1; j++)
		{
			printf("%f ", X_k[i][j]);
		}
		printf("\n");
	}
}



#endif /* INC_KALMANFILTER_H_ */
