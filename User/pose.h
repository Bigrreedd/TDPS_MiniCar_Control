#ifndef POSE_H_
#define POSE_H_
//????????
typedef struct
{
    float x;
    float y;
    float z;
}float_xyz_struct;

//???????????
typedef struct
{
    float rol;
    float pit;
    float yaw;
}float_ang_struct;
extern float q0, q1, q2, q3;
extern float_ang_struct    att_angle;
extern float_xyz_struct    gyr_rad,gyr_radold;               //??????????????????????????????????
extern float_xyz_struct    acc_g,gry_filt,acc_gold;    //??????????????
extern void prepare_data(void);
extern void imuupdate(float_xyz_struct *gyr_rad,float_xyz_struct *acc_g,float_ang_struct *att_angle);
extern void one_filter(float_xyz_struct *acc,float_xyz_struct *gyro, float_xyz_struct *filter_angle);
#endif
