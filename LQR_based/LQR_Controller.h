

class LQR_Controller
{
public:
  LQR_Controller(const float * A, const float * B, const float* F, float *x, float *xd, float * u, int n, int p);
  ~LQR_Controller();

  float* Cal_Control_Input();

  void MatrixMultiplication(const float *left, const float* right, float *res, int row, int col1, int col2);
  void MatrixAdd(const float *left, const float* right, float *res, int row, int col);
  void MatrixMinus(const float *left, const float* right, float *res, int row, int col);

private:
  float * m_A;
  float * m_B;
  float * m_x;
  //float m_xa[8];
  float * m_xd;
  float * m_u;
  //control input
  float PWM_Out;
  float * m_Feedback_Gain;
  int m_n, m_p;
};
