#include "LQR_Controller.h"

LQR_Controller::LQR_Controller(const float * A, const float * B, const float* F, float *x, float *xd, float * u, int n, int p)
{
    m_A = A;//n x n
    m_B = B;//n x p
    m_x = x;
    m_xd = xd;//n x 1
    m_u = u;//p x 1

    m_n = n;
    m_p = p;
    m_Feedback_Gain = F;//p x n
}


LQR_Controller::~LQR_Controller()
{
//    for (int i = 0; i < p; i++)
//      delete[] Feedback_Gain[i];
//    delete[] Feedback_Gain;
//
//    for (int i = 0; i < n; i++)
//      delete[] m_A[i];
//    delete[] m_A;
//
//    for (int i = 0; i < n; i++)
//      delete[] m_B[i];
//    delete[] m_B;

    delete m_Feedback_Gain;
    delete m_A;
    delete m_B;

    delete m_x;
    //delete m_xa;
    delete m_xd;
    delete m_u;
}

float* LQR_Controller::Cal_Control_Input()
{
  
    MatrixMultiplication(m_Feedback_Gain, m_x, m_u, m_p, m_n, 1);
//    float *Ax = new float(m_n*1);
//    MatrixMultiplication(m_A, m_x, Ax, m_n, m_n, 1);
//    float *Bu = new float(m_n*1);
//    MatrixMultiplication(m_B, m_u, Bu, m_n, m_p, 1);
//    MatrixMinus(Ax, Bu, m_x);
    return m_u;
}

void LQR_Controller::MatrixMultiplication(const float *left, const float* right, float *res, int row, int col1, int col2)
{
  int i = 0, j = 0;
  for (int idx = 0; idx < row * col2; idx++)
  {
    i = idx / col2;
    j = idx - i * col2;
    for (int k = 0; k < col1; k++)
    {
      res[idx] = res[idx] + left[i * col1 + k] * right[j + k * col2];
    }
  }
}

void LQR_Controller::MatrixAdd(const float *left, const float* right, float *res, int row, int col)
{
    for (int i = 0; i < row; i++)
    {
      for (int j = 0; j < col; j++)
      {
          res[j + i * col] = left[i * col + j] + right[i * col + j];
      }  
    }
}

void LQR_Controller::MatrixMinus(const float *left, const float* right, float *res, int row, int col)
{
    for (int i = 0; i < row; i++)
    {
      for (int j = 0; j < col; j++)
      {
          res[j + i * col] = left[i * col + j] - right[i * col + j];
      }  
    }
}
