//
// Created by quonone on 23-2-22.
//

#ifndef KALMANFILTER_KALMANFILTER_H
#define KALMANFILTER_KALMANFILTER_H
#include <eigen3/Eigen/Eigen>


namespace ares{
    class KalmanFilter {
    public:
        KalmanFilter( Eigen::MatrixXd& A_,  Eigen::MatrixXd& P_, \
     Eigen::MatrixXd& H_,  Eigen::MatrixXd& Q_,  Eigen::MatrixXd& R_):
                A(A_), P0(P_), H(H_), Q(Q_), R(R_),
                m(H.rows()), n(A.rows()), I(n,n),X_hat(n), X_hat_new(n){
            I.setIdentity(n,n);

        }
        ~KalmanFilter()=default;

        Eigen::VectorXd X_hat_new; //最优估计
        Eigen::MatrixXd P; //协方差矩阵    1
        Eigen::MatrixXd H; //测量矩阵      1
        Eigen::MatrixXd Q; //过程噪声     1
        Eigen::MatrixXd R; //测量噪声     1
        //初始状态量，当前时刻
        void init(const Eigen::VectorXd& X0,double t){
            P = P0;
            X_hat = X0;
            this->t0 = t;
            this->last_t = t0;
        }
        //先predict，在value，最后获得最优估计
        void predict(const Eigen::VectorXd& Y, double t){
            double dt = t - last_t;
            A  << 1,0,0,dt,0,0,
                    0,1,0,0,dt,0,
                    0,0,1,0,0,dt,
                    0,0,0,1,0,0,
                    0,0,0,0,1,0,
                    0,0,0,0,0,1;
            last_t = t;
            //predict
            X_hat_new = A*X_hat;

        }
        void update(const Eigen::VectorXd& Y){
            P = A*P*A.transpose() + Q;
            //update
            K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
            X_hat_new = X_hat_new + K*(Y - H*X_hat_new);
            P = (I - K*H)*P;
            //
            X_hat = X_hat_new;
        }

    private:
        Eigen::MatrixXd A; //状态转移矩阵  1
        Eigen::MatrixXd B; //控制矩阵      1

        Eigen::MatrixXd P0; //协方差矩阵    1
        Eigen::MatrixXd K; //卡尔曼增益/

        Eigen::MatrixXd I; //单位


        Eigen::VectorXd X_hat; //先验估计，列向量

        int m,n;
        double t0, last_t;
    };

};


#endif //KALMANFILTER_KALMANFILTER_H
