#include "pos_localization.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "system.h"
#include "task.h"
#include "arm_math.h"
#include "uwbp2mdist.h"
#include "math.h"
#define num_UAV 3

static bool isInit = false;
static uint16_t idVelocityX;
static uint16_t idVelocityY;
static uint16_t idPositionZ;
static uint16_t idyawRate;
static uint16_t idIsFlying;


static position_state position[num_UAV];
static position_state *myPos;

static float velocityX,velocityY,positionZ;
static float yawRate;
static float HPHR;
static uint32_t osTick;
static float esti_vec_ang,esti_distance_ang,residual_ang;
static float dtEKF;
static bool isFlying;
static float position_x=0.0;
static float position_y=0.0;
static float position_z=0.0;
static float position_yaw=0.0;
static bool isRefresh[num_UAV]={false};
static float InitCovPos = 1000.0f;
static float InitCovYaw = 1.0f;
static uint8_t my_addr_idx;
static float Qv = 0.05f; // velocity deviation
static float Qr = 0.1f; // yaw rate deviation
static float Ruwb = 0.03f; // ranging deviation
static bool isAnchor[num_UAV] = {true,false,true};
static float init_position[4][num_UAV]={
                                    {0.0,1,0.0},//x
                                    {0.0,1,1.0},//y
                                    {0.8,0.0,0.8},//z
                                    {0.0,0.0,0.0},//yaw
                                };// also contains anchor's position


static position_state pos_predict;
// static arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)pos_predict.P};

static float B[STATE_DIM_rl][STATE_DIM_rl];
static arm_matrix_instance_f32 Bm = {STATE_DIM_rl, STATE_DIM_rl, (float *)B};

static float Q[STATE_DIM_rl][STATE_DIM_rl];
static arm_matrix_instance_f32 Qm = {STATE_DIM_rl, STATE_DIM_rl, (float *)Q};


static float tmp1[STATE_DIM_rl][STATE_DIM_rl];
static arm_matrix_instance_f32 tmp1m = { STATE_DIM_rl, STATE_DIM_rl, (float *)tmp1};

static float tmp2[STATE_DIM_rl][STATE_DIM_rl];
static arm_matrix_instance_f32 tmp2m = { STATE_DIM_rl, STATE_DIM_rl, (float *)tmp2};

static float tmp3[4];
static arm_matrix_instance_f32 tmp3m = {1,4,(float *)tmp3};

static float tmp4[4][4];
static arm_matrix_instance_f32 tmp4m = {4,4,(float*) tmp4};

static float tmp5[4][4];
static arm_matrix_instance_f32 tmp5m = {4,4,(float*) tmp5};

static float H[4];
static arm_matrix_instance_f32 Hm = {1,4,(float *) H};

static float Ht[4];
static arm_matrix_instance_f32 Htm = {4,1,(float *) Ht};

static float combine_P[4][4];
static arm_matrix_instance_f32 combine_Pm = {4,4,(float *) combine_P};

static float Ss;


static float K[4];
static arm_matrix_instance_f32 Km = {4,1,(float *) K};

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }


uint8_t initLocalization(){
    if (isInit){
        return my_addr_idx;
    }
    vTaskDelay(10000);
    idVelocityX = logGetVarId("stateEstimateZ", "vx");
    idVelocityY = logGetVarId("stateEstimateZ", "vy");
    idPositionZ = logGetVarId("stateEstimateZ","z");
    idyawRate = logGetVarId("stateEstimateZ", "rateYaw");
    idIsFlying = logGetVarId("kalman","inFlight");
    my_addr_idx = getmy_idx();
    myPos = &position[my_addr_idx];
   
    for (int i=0; i<STATE_DIM_rl; i++) {
      for (int j=0; j<STATE_DIM_rl; j++) {
        Q[i][j] = 0;
      }
    }
    Q[STATE_rlX][STATE_rlX] = Qv;
    Q[STATE_rlY][STATE_rlY] = Qv;
    Q[STATE_rlYaw][STATE_rlYaw] = Qr;
    // init position
    for (size_t i = 0; i < num_UAV; i++)
    {
        position[i].x = init_position[0][i];
        position[i].y = init_position[1][i];
        position[i].z = init_position[2][i];
        position[i].yaw = init_position[3][i];
        for (int ii=0; ii<STATE_DIM_rl; ii++) {
            for (int jj=0; jj<STATE_DIM_rl; jj++) {
                position[i].P[ii][jj] = 0;
            }     
        }
        position[i].P[STATE_rlX][STATE_rlX] = InitCovPos;
        position[i].P[STATE_rlY][STATE_rlY] = InitCovPos;
        position[i].P[STATE_rlYaw][STATE_rlYaw] = InitCovYaw;
    }

    
    vTaskDelay(10000);
    // xTaskCreate(localizationTask,"Localization",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
    isInit = true;
    return my_addr_idx;
}


float angle_trans(float angle){
    float ans;
    ans = angle + 2*PI;
    ans =fmodf(ans,2*PI);
    //trans anglo from [-pi,pi]
    if (ans > PI){
        ans = ans - 2*PI;
    }else{
        //do nothing
    }
    return ans;
}

void localizationTask(){
    systemWaitStart();
    
    myPos->old_tick = xTaskGetTickCount();
    position_x = myPos->x;
    position_y = myPos->y;
    position_z = myPos->z;
    
    //estimate self postion
    while(1) {
        vTaskDelay(100);
    
        //get information
        velocityX = logGetFloat(idVelocityX)/1000; //mm/sec -> m/sec
        velocityY = logGetFloat(idVelocityY)/1000; //mm/sec -> m/sec
        positionZ = logGetFloat(idPositionZ)/1000; // mm -> m
        isFlying = logGetUint(idIsFlying);
        if (!isFlying)
        {
            velocityX = 0.0f;
            velocityY = 0.0f;
            
        }
        //estimate possible direction for velocity
        esti_vec_ang = atan2f(velocityY,velocityX) + myPos->yaw; 
        esti_vec_ang = angle_trans(esti_vec_ang);
        // DEBUG_PRINT("velocityY:%f velocityX:%f esti_vec_ang:%f\n",(double)velocityY,(double)velocityX,(double)esti_vec_ang);

        yawRate = logGetInt(idyawRate)/1000.0f;// milliradians / sec -> radians/sec
        osTick = xTaskGetTickCount();
        dtEKF  = (float)(osTick - myPos->old_tick)/configTICK_RATE_HZ;
        myPos->old_tick = osTick;
        
        // DEBUG_PRINT("1 x:%f y:%f dis:%f\n",(double)position[1].x,(double)position[1].y,(double)position[1].distance);
        // DEBUG_PRINT("2 x:%f y:%f dis:%f\n",(double)position[2].x,(double)position[2].y,(double)position[2].distance);
        // DEBUG_PRINT("3 x:%f y:%f dis:%f\n",(double)position[3].x,(double)position[3].y,(double)position[3].distance);
        //run EKF
        if(isAnchor[my_addr_idx]){
            myPos->x = init_position[0][my_addr_idx];
            myPos->y = init_position[1][my_addr_idx];
            myPos->z = init_position[2][my_addr_idx];
            myPos->yaw= init_position[3][my_addr_idx];
            myPos->P[0][0] = 0.1f;
            myPos->P[1][1] = 0.1f;
            //tell others
            getpositionInfo(position,isRefresh,num_UAV);

        }else{
            //set z 
            myPos->z = positionZ;
            getpositionInfo(position,isRefresh,num_UAV);
            int refreshcnt = 0;
            for (uint8_t i = 0; i < num_UAV; i++)
            {
                if (isRefresh[i]){
                    refreshcnt += 1;
                }
            }
            
            EKF(velocityX,velocityY,yawRate,dtEKF);
            esti_distance_ang = atan2f(myPos->y-position_y,myPos->x-position_x);
            if (powf(velocityX,2)+powf(velocityY,2)>0.01f){
                residual_ang = (esti_distance_ang-esti_vec_ang);
                residual_ang = angle_trans(residual_ang);
                DEBUG_PRINT("dy:%f dx:%f vx:%f vy:%f \n",(double)(myPos->y-position_y),(double)(myPos->x-position_x),(double)velocityX,(double)velocityY);
                DEBUG_PRINT("ref_cnt:%d dis_ang:%f ve_ang:%f res:%f\n",refreshcnt,(double)esti_distance_ang,(double)esti_vec_ang,(double)residual_ang);
                if (residual_ang){

                }else{
                    /* code */
                }
                
                // myPos->yaw = myPos->yaw + 0.05f*residual_ang;
                myPos->yaw = myPos->yaw +residual_ang*(myPos->P[2][2])/(myPos->P[2][2]+1);
                myPos->P[2][2] = myPos->P[2][2]-(myPos->P[2][2]*myPos->P[2][2])/(myPos->P[2][2]+1);
            }
            myPos->yaw = angle_trans(myPos->yaw);
        }
        position_x = myPos->x;
        position_y = myPos->y;
        position_z = myPos->z;
        position_yaw = myPos->yaw;
        
    }

}


static float cyaw,syaw;
static float dt;
static float observation,zPred,resErr;
void EKF(float vx,float vy,float yawRate,float dtEKF){
    
    cyaw = arm_cos_f32(myPos->yaw);
    syaw = arm_sin_f32(myPos->yaw);
    // cyaw = cosf(myPos->yaw);
    // syaw = sinf(myPos->yaw);
    // DEBUG_PRINT("vx:%f vy:%f cyaw:%f syaw:%f\n",(double)vx,(double)vy,(double)cyaw,(double)syaw);
    /********************predict state**************/
    // float pos_cov_predict[3][3]; covariance prediction
    pos_predict.x = myPos->x + (cyaw*vx-syaw*vy)*dtEKF;
    pos_predict.y = myPos->y + (syaw*vx+cyaw*vy)*dtEKF;
    pos_predict.z = myPos->z;
    pos_predict.yaw = myPos->yaw + yawRate*dtEKF;
    /********************predict state end**************/
    // DEBUG_PRINT("cyaw*vx:%f syaw*vy:%f (cyaw*vx-syaw*vy)*dtEKF:%f\n",(double)(cyaw*vx),(double)(syaw*vy),(double)(cyaw*vx-syaw*vy));
    // DEBUG_PRINT("before x:%f y:%f yaw:%f\n", (double)myPos->x,(double)myPos->y,(double)myPos->yaw);
    // DEBUG_PRINT("after x:%f y:%f yaw:%f\n", (double)pos_predict.x,(double)pos_predict.y,(double)pos_predict.yaw);
    
    /**************** update covariance***************/
    // B
    B[0][0] = cyaw;
    B[0][1] = -syaw;
    B[0][2] = 0;
    B[1][0] = syaw;
    B[1][1] = cyaw;
    B[1][2] = 0;
    B[2][0] = 0;
    B[2][1] = 0;
    B[2][2] = 1;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("B:%f %f %f\n",(double)B[i][0],(double)B[i][1],(double)B[i][2]);    
    // }

    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("Q:%f %f %f\n",(double)Q[i][0],(double)Q[i][1],(double)Q[i][2]);    
    // }
    
    // BQB'
    mat_mult(&Bm, &Qm, &tmp1m); // B Q
    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("tmp1:%f %f %f\n",(double)tmp1[i][0],(double)tmp1[i][1],(double)tmp1[i][2]);    
    // }

    mat_trans(&Bm, &tmp2m); // B'
    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("tmp2:%f %f %f\n",(double)tmp2[i][0],(double)tmp2[i][1],(double)tmp2[i][2]);    
    // }

    mat_mult(&tmp1m, &tmp2m, &Bm); // B Q B'
    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("B Q B':%f %f %f\n",(double)B[i][0],(double)B[i][1],(double)B[i][2]);    
    // }
    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("myPos->P:%f %f %f\n",(double)myPos->P[i][0],(double)myPos->P[i][1],(double)myPos->P[i][2]);    
    // }

    // P_predict = P + BQB'
    dt = dtEKF*dtEKF;
    pos_predict.P[0][0] = myPos->P[0][0]+ B[0][0]*dt;
    pos_predict.P[0][1] = myPos->P[0][1]+ B[0][1]*dt;
    pos_predict.P[0][2] = myPos->P[0][2]+ B[0][2]*dt;
    pos_predict.P[1][0] = myPos->P[1][0]+ B[1][0]*dt;
    pos_predict.P[1][1] = myPos->P[1][1]+ B[1][1]*dt;
    pos_predict.P[1][2] = myPos->P[1][2]+ B[1][2]*dt;
    pos_predict.P[2][0] = myPos->P[2][0]+ B[2][0]*dt;
    pos_predict.P[2][1] = myPos->P[2][1]+ B[2][1]*dt;
    pos_predict.P[2][2] = myPos->P[2][2]+ B[2][2]*dt;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     DEBUG_PRINT("pos_predict.P:%f %f %f\n",(double)pos_predict.P[i][0],(double)pos_predict.P[i][1],(double)pos_predict.P[i][2]);    
    // }

    /**************** update covariance done***************/


    /********** add observation **********/
    /****** add anchor observation******/
    /****** add UWB observation(only update x,y.) *****/
    HPHR = powf(Ruwb, 2);//noisy R a diag matrix;
    
    for (int i = 0; i < num_UAV; i++)
    {
        if (i!= my_addr_idx && isRefresh[i]){
            // DEBUG_PRINT("observe i:%d distance:%f\n",i,(double)position[i].distance);
            // observation(UWB distance)
            observation = position[i].distance;
            // prediction for observation
            zPred = sqrtf(powf(pos_predict.x-position[i].x,2)+powf(pos_predict.y-position[i].y,2)+powf(pos_predict.z-position[i].z,2));
            // residual error
            resErr= observation-zPred;
            // DEBUG_PRINT("zPred:%f resErr:%f\n",(double)zPred,(double)resErr);
            // establish H, the combined status
            H[0]=(pos_predict.x-position[i].x)/(zPred+0.0001f);
            H[1]=(pos_predict.y-position[i].y)/(zPred+0.0001f);
            H[2]=-H[0];
            H[3]=-H[1];
            // establish combined_P,the combined covariance
            /**
             *
             * combineP:[pos_predict.P(2*2) , 0 ,0]
             *          [pos_predict.P(2*2) , 0 ,0]
             *          [0, 0 ,position.covx ,0]
             *          [0, 0 ,0 ,position.covy]
             *  
             */
            combine_P[0][0] = pos_predict.P[0][0];
            combine_P[0][1] = pos_predict.P[0][1];
            combine_P[0][2] = 0;
            combine_P[0][3] = 0;

            combine_P[1][0] = pos_predict.P[1][0];
            combine_P[1][1] = pos_predict.P[1][1];
            combine_P[1][2] = 0;
            combine_P[1][3] = 0;

            combine_P[2][0] = 0;
            combine_P[2][1] = 0;
            combine_P[2][2] = position[i].P[0][0];
            combine_P[2][3] = 0;

            combine_P[3][0] = 0;
            combine_P[3][1] = 0;
            combine_P[3][2] = 0;
            combine_P[3][3] = position[i].P[1][1];
            /***********combine_p done********/
            
            //Ss = jacoH * combine_P* jacoH' + R
            mat_trans(&Hm, &Htm); // jacoH'(jacoH':4*1)
            // DEBUG_PRINT("H:%f %f %f %f\n",(double)H[0],(double)H[1],(double)H[2],(double)H[3]);
            // DEBUG_PRINT("Ht:%f %f %f %f\n",(double)Ht[0],(double)Ht[1],(double)Ht[2],(double)Ht[3]);
            // for (size_t j = 0; j < 4; j++)
            // {
            //     DEBUG_PRINT("combine_Pm:%f %f %f %f\n",(double)combine_P[j][0],(double)combine_P[j][1],(double)combine_P[j][2],(double)combine_P[j][3]);
            // }
            
            mat_mult( &combine_Pm,&Htm, &tmp3m); // combine_P * jacoH' (combine_P:4*4)*(jacoH':4*1)
            // DEBUG_PRINT("tmp3m:%f %f %f %f\n",(double)tmp3[0],(double)tmp3[1],(double)tmp3[2],(double)tmp3[3]);
            
            
            Ss=0;
            for (int j = 0;j<4 ;j++){
                Ss = Ss + H[j]*tmp3[j];//jacoH * combine_P* jacoH'
            }
            Ss = Ss + HPHR; //jacoH * combine_P* jacoH' + R
            // DEBUG_PRINT("Ss:%f\n",(double)Ss);
            for (int j = 0; j < 4; j++)
            {
                K[j] = tmp3[j]/Ss;  // kalman gain = (combine_P * jacoH' (Ss )^-1)
            }
            /********update state and covariance ********/
            // DEBUG_PRINT("before pos_predict.x:%f pos_predict.y:%f position[i].x:%f position[i].y:%f zPred:%f\n",(double)(pos_predict.x),(double)(pos_predict.y),(double)position[i].x,(double)position[i].y,(double)zPred);
            pos_predict.x = pos_predict.x + K[0]*resErr;//update x
            pos_predict.y = pos_predict.y + K[1]*resErr;//update y 
            // DEBUG_PRINT("after pos_predict.x:%f pos_predict.y:%f K[0]:%f K[1]:%f resErr:%f\n",(double)(pos_predict.x),(double)(pos_predict.y),(double)K[0],(double)K[1],(double)resErr);
            
            mat_mult(&Km,&Hm,&tmp4m);//  KH
            for (int j =0;j<4;j++){
                for (int k=0;k<4;k++){
                    //  I-KH
                    if (j == k){
                        tmp4[j][k] = 1-tmp4[j][k];
                    }else{
                        tmp4[j][k] = -tmp4[j][k];
                    }
                    
                }
            }
            //
            
            //copy combine_P to tmp5
            tmp5[0][0] = combine_P[0][0];
            tmp5[0][1] = combine_P[0][1];
            tmp5[0][2] = combine_P[0][2];
            tmp5[0][3] = combine_P[0][3]; 
            tmp5[1][0] = combine_P[1][0];
            tmp5[1][1] = combine_P[1][1];
            tmp5[1][2] = combine_P[1][2];
            tmp5[1][3] = combine_P[1][3];
            tmp5[2][0] = combine_P[2][0];
            tmp5[2][1] = combine_P[2][1];
            tmp5[2][2] = combine_P[2][2];
            tmp5[2][3] = combine_P[2][3];
            tmp5[3][0] = combine_P[3][0];
            tmp5[3][1] = combine_P[3][1];
            tmp5[3][2] = combine_P[3][2];
            tmp5[3][3] = combine_P[3][3];
            mat_mult(&tmp4m,&tmp5m,&combine_Pm);//(I-KH)*P

            //update covariance
            pos_predict.P[0][0] = combine_P[0][0];
            pos_predict.P[0][1] = combine_P[0][1];
            pos_predict.P[1][0] = combine_P[1][0];
            pos_predict.P[1][1] = combine_P[1][1];
        }else{
            continue;
        }
    }
    
    /********** add observation done**********/








    // finally, update my position
    myPos->x = pos_predict.x;
    myPos->y = pos_predict.y;
    myPos->yaw = pos_predict.yaw;
    // DEBUG_PRINT("pos_predict.x:%f pos_predict.y:%f\n",(double)(pos_predict.x),(double)(pos_predict.y));
    myPos->P[0][0] = pos_predict.P[0][0];
    myPos->P[0][1] = pos_predict.P[0][1];
    myPos->P[0][2] = pos_predict.P[0][2];
    myPos->P[1][0] = pos_predict.P[1][0];
    myPos->P[1][1] = pos_predict.P[1][1];
    myPos->P[1][2] = pos_predict.P[1][2];
    myPos->P[2][0] = pos_predict.P[2][0];
    myPos->P[2][1] = pos_predict.P[2][1];
    myPos->P[2][2] = pos_predict.P[2][2];
}

LOG_GROUP_START(poslocation)
LOG_ADD(LOG_FLOAT, posx,&position_x)
LOG_ADD(LOG_FLOAT, posy,&position_y)
LOG_ADD(LOG_FLOAT, posz,&position_z)
LOG_ADD(LOG_FLOAT,posyaw,&position_yaw)
LOG_GROUP_STOP(poslocation)