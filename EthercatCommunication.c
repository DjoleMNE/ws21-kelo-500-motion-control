/**
 * @file EthercatCommunication.c
 * @author Kavya Shankar (kavya.shankar@smail.inf.h-brs.de)
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 * @date 2022-03-12
 * 
 */
#include "ethercat.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
#include "KeloDriveAPI.h"
#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_sf_trig.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit.h>
#include <unistd.h>
#include "PlatformToWheelInverseKinematicsSolver.h"
#include "KELORobotKinematics.h"
#include "abag.h"
#include <gsl/gsl_matrix_double.h>
#include <string.h> 

#define SIM_CONTROL 0

/**
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 * 
 * @param argc 
 * @param argv 
 * @return int to signify successful execution of the function
 */
int main(int argc, char *argv[])
{
    int NUM_DRIVES = 4; 
    int index_to_EtherCAT[4] = {2, 6, 10, 11};
    bool debug = false;
    // char arg[] = "debug";
    // if (strcmp(argv[1],arg) == 0)
    // {
    //     debug = true;
    // }

#if SIM_CONTROL == 0

    rxpdo1_t msg;
    msg.timestamp = 1;
    msg.command1 = 0;
    msg.limit1_p = 0;
    msg.limit1_n = 0;
    msg.limit2_p = 0;
    msg.limit2_n = 0;
    msg.setpoint1 = 0;
    msg.setpoint2 = 0;

    ec_slavet ecx_slave[EC_MAXSLAVE];
    int ecx_slavecount;
    ec_groupt ec_group[EC_MAXGROUP];
    uint8 esibuf[EC_MAXEEPBUF];
    uint32 esimap[EC_MAXEEPBITMAP];
    ec_eringt ec_elist;
    ec_idxstackT ec_idxstack;

    ec_SMcommtypet ec_SMcommtype;
    ec_PDOassignt ec_PDOassign;
    ec_PDOdesct ec_PDOdesc;
    ec_eepromSMt ec_SM;
    ec_eepromFMMUt ec_FMMU;
    boolean EcatError;
    int64 ec_DCtime;
    ecx_portt ecx_port;
    ecx_redportt ecx_redport;
    ecx_contextt ecx_context;
    char IOmap[4096];

    ecx_context.port = &ecx_port;
    ecx_context.slavelist = &ecx_slave[0];
    ecx_context.slavecount = &ecx_slavecount;
    ecx_context.maxslave = EC_MAXSLAVE;
    ecx_context.grouplist = &ec_group[0];
    ecx_context.maxgroup = EC_MAXGROUP;
    ecx_context.esibuf = &esibuf[0];
    ecx_context.esimap = &esimap[0];
    ecx_context.esislave = 0;
    ecx_context.elist = &ec_elist;
    ecx_context.idxstack = &ec_idxstack;

    ecx_context.ecaterror = &EcatError;
    ecx_context.DCtime = &ec_DCtime;
    ecx_context.SMcommtype = &ec_SMcommtype;
    ecx_context.PDOassign = &ec_PDOassign;
    ecx_context.PDOdesc = &ec_PDOdesc;
    ecx_context.eepSM = &ec_SM;
    ecx_context.eepFMMU = &ec_FMMU;
    ecx_context.manualstatechange = 0; // should be 0

    /**
     * @brief port name on our PC to initiate connection
     * 
     */
    if (!ecx_init(&ecx_context, "enp0s31f6"))
    { 
        printf("Failed to initialize EtherCAT\n");
        return 0;
    }

    /**
     * @brief checking establishment of first connection with slave or autoconfig slaves
     * 
     */
    if (!ecx_config_init(&ecx_context, TRUE))
    { 
        printf("NO SLAVES!\n");
        return 0;
    }
    ecx_config_map_group(&ecx_context, IOmap, 0); // PDO - process data object

    printf("%i\n", ecx_slavecount);
    printf("%s\n", ecx_slave[1].name);

    /**
     * @brief Reading all slave names w.r.t their no.
     * 
     */
    for (int i = 1; i <= ecx_slavecount; i++)
    {
        printf("slave \t%i has name \t%s\n", i, ecx_slave[i].name);
    }

    /**
     * @brief waiting for all slaves to reach SAFE_OP state
     * 
     */
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    if (ecx_slave[0].state != EC_STATE_SAFE_OP)
    {
        printf("EtherCAT slaves have not reached safe operational state\n");
        ecx_readstate(&ecx_context);

        /**
         * @brief if not all slaves operational, find out which one
         * 
         */
        for (int i = 1; i <= ecx_slavecount; i++)
        {
            if (ecx_slave[i].state != EC_STATE_SAFE_OP)
            {
                printf("Slave %i State= %i\n", i, ecx_slave[i].state);
            }
        }
        return 0;
    }

    for (unsigned int i = 0; i < NUM_DRIVES; i++)
    {
        rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
        *ecData = msg;
    }

    /**
     * @brief sending process data
     * 
     */
    ecx_send_processdata(&ecx_context); 

    /**
     * @brief setting state to operational
     * 
     */
    ecx_slave[0].state = EC_STATE_OPERATIONAL; 

    /**
     * @brief receiving response from slaves
     * 
     */
    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

    ecx_writestate(&ecx_context, 0);

    /**
     * @brief checking if the slaves have reached operational state
     * 
     */
    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); 

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("EtherCAT slaves have not reached operational state\n");
        return 0;
    }
    else
    {
        printf("Operational state reached for all EtherCAT slaves.\n");
    }
#endif

    /**
     * @brief initialising pointers to variables used for solving the problem of inverse kinematics
     * 
     */
    const unsigned int N = 3;
    const unsigned int M = 8;
    double motor_const = 0.29; //units: (Newton-meter/Ampere)
    double max_current = 10.0; // units Ampere
    gsl_matrix *A = gsl_matrix_alloc(N, M);
    gsl_matrix *A_inv_T = gsl_matrix_alloc(M, N);
    gsl_matrix *A_tmp = gsl_matrix_alloc(N, M);
    gsl_matrix *A_inv_T_tmp = gsl_matrix_alloc(M, N);
    gsl_vector *work = gsl_vector_alloc(N);
    gsl_matrix *W = gsl_matrix_alloc(N, N); 
    gsl_matrix *K = gsl_matrix_alloc(M, M); 
    gsl_vector *u = gsl_vector_alloc(N);
    gsl_matrix *V = gsl_matrix_alloc(N, N);
    gsl_matrix *u_inv = gsl_matrix_alloc(N, N);
    gsl_matrix *b = gsl_matrix_alloc(N, 1);
    gsl_matrix *b_verify = gsl_matrix_alloc(N, 1);

    // ABAG parameters
    const double alpha_parameter[4]          = { 0.900000, 0.900000, 0.950000, 0.950000 };
    const double bias_threshold_parameter[4] = { 0.000407, 0.000407, 0.000407, 0.000407 };
    const double bias_step_parameter[4]      = { 0.000400, 0.000400, 0.000400, 0.000400 };
    const double gain_threshold_parameter[4] = { 0.550000, 0.550000, 0.550000, 0.550000 };
    const double gain_step_parameter[4]      = { 0.003000, 0.003000, 0.003000, 0.003000 };
    double desired_state[4] = { 0.0, 0.0, 0.0, 0.0 };
    double ctrl_error[4]    = { 0.0, 0.0, 0.0, 0.0 };
    double abag_command[4]  = { 0.0, 0.0, 0.0, 0.0 };
    abagState_t abag_state[4];
    for (int i = 0; i < 4; i++)
    {
        initialize_abagState(&abag_state[i]);
    }

    /**
     * @brief initialising arrays to store pivot angles and wheel torques
     * 
     */
    double pivot_angles[4]  = {0.0, 0.0, 0.0, 0.0};
    double wheel_torques[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /**
     * @brief setting input platform force values
     * 
     */
    gsl_matrix_set(b, 0, 0, 90.0); // force is set in X-direction
    gsl_matrix_set(b, 1, 0, 0.0); // force is set in Y-direction
    gsl_matrix_set(b, 2, 0, 0.0); // moment is set in anti-clockwise direction

#if SIM_CONTROL == 0
    /**
     * @brief reading data from individual wheels
     * 
     */
    for (unsigned int i = 0; i < NUM_DRIVES; i++)
    {
        txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
        pivot_angles[i] = ecData->encoder_pivot;
    }
#endif

    /**
     * @brief setting the weight matrix
     * 
     */
    size_t i;
    for (i = 0; i < M; i++)
    {

        gsl_matrix_set(K, i, i, 1.0);
        if (i < N)
        {
            gsl_matrix_set(W, i, i, 1.0);
        }
    }

    /**
     * @brief setting number of iterations until which the force has to be applied
     * 
     */
    int loop_count = 0;
    while (loop_count < 5000000)
    {
        /**
         * @brief setting sleep time between iterations to achieve communication frequency of 1000Hz
         * 
         */
        usleep(1000);

        /**
         * @brief finding wheel torques for each iteration parameterised by pivot angles 
         * 
         */
        functions_main(wheel_torques,
                       pivot_angles,
                       b,
                       b_verify,
                       A,
                       A_inv_T,
                       A_tmp,
                       A_inv_T_tmp,
                       work,
                       W,
                       K,
                       u,
                       V,
                       u_inv,
                       M,
                       N,
                       debug);

        for (int i = 0; i < NUM_DRIVES; i++)
        {
            if (pivot_angles[i] > M_PI) pivot_angles[i] -= 2 * M_PI;

            ctrl_error[i] = desired_state[i] - pivot_angles[i];
            if (fabs(ctrl_error[i]) < 0.15) ctrl_error[i] = 0.0;
        }
        // printf("Pivot angles: %f, %f, %f, %f\n", pivot_angles[0], pivot_angles[1], pivot_angles[2], pivot_angles[3]);
        // printf("Error: %f, %f, %f, %f\n", ctrl_error[0], ctrl_error[1], ctrl_error[2], ctrl_error[3]);

        // abag_sched(&abag_state[0], &ctrl_error[0],
        //            &abag_command[0], &alpha_parameter[0],
        //            &bias_threshold_parameter[0], &bias_step_parameter[0],
        //            &gain_threshold_parameter[0], &gain_step_parameter[0]);
        // printf("error: %f, bias: %f, gain: %f, cmd: %f\n\n", abag_state[0].eBar_access, 
        //        abag_state[0].bias_access, abag_state[0].gain_access, abag_command[0]);
        // wheel_torques[0] = abag_command[0] * max_current * motor_const;
        // wheel_torques[1] = -abag_command[0] * max_current * motor_const;
        // wheel_torques[0] = 0.0;
        // wheel_torques[1] = 0.0;

        // abag_sched(&abag_state[1], &ctrl_error[1],
        //            &abag_command[1], &alpha_parameter[1],
        //            &bias_threshold_parameter[1], &bias_step_parameter[1],
        //            &gain_threshold_parameter[1], &gain_step_parameter[1]);
        // printf("error: %f, bias: %f, gain: %f, cmd: %f\n\n", abag_state[0].eBar_access, 
        //        abag_state[0].bias_access, abag_state[0].gain_access, abag_command[0]);
        // wheel_torques[2] = abag_command[1] * max_current * motor_const;
        // wheel_torques[3] = -abag_command[1] * max_current * motor_const;
        // wheel_torques[2] = 0.0;
        // wheel_torques[3] = 0.0;

        // abag_sched(&abag_state[2], &ctrl_error[2],
        //            &abag_command[2], &alpha_parameter[2],
        //            &bias_threshold_parameter[2], &bias_step_parameter[2],
        //            &gain_threshold_parameter[2], &gain_step_parameter[2]);
        // printf("error: %f, bias: %f, gain: %f, cmd: %f\n\n", abag_state[0].eBar_access, 
        //        abag_state[0].bias_access, abag_state[0].gain_access, abag_command[0]);
        // wheel_torques[4] = abag_command[2] * max_current * motor_const;
        // wheel_torques[5] = -abag_command[2] * max_current * motor_const;
        // wheel_torques[4] = 0.0;
        // wheel_torques[5] = 0.0;

        // abag_sched(&abag_state[3], &ctrl_error[3],
        //            &abag_command[3], &alpha_parameter[3],
        //            &bias_threshold_parameter[3], &bias_step_parameter[3],
        //            &gain_threshold_parameter[3], &gain_step_parameter[3]);
        // printf("error: %f, bias: %f, gain: %f, cmd: %f\n\n", abag_state[0].eBar_access, 
        //        abag_state[0].bias_access, abag_state[0].gain_access, abag_command[0]);
        // wheel_torques[6] = abag_command[3] * max_current * motor_const;
        // wheel_torques[7] = -abag_command[3] * max_current * motor_const;
        // wheel_torques[6] = 0.0;
        // wheel_torques[7] = 0.0;

        printf("Torque: %f, %f, %f, %f, %f, %f, %f, %f\n\n", wheel_torques[0], wheel_torques[1],
                                                             wheel_torques[2], wheel_torques[3],
                                                             wheel_torques[4], wheel_torques[5],
                                                             wheel_torques[6], wheel_torques[7]);

        rxpdo1_t msg;
        msg.timestamp = time(NULL);
        msg.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_TORQUE;
        msg.limit1_p = 0.0;  // Wheel's microcontroller does not seem to use those limits in torque control mode
        msg.limit1_n = 0.0;
        msg.limit2_p = 0.0;
        msg.limit2_n = 0.0;

        /**
         * @brief setting calculated torque values to individual wheels
         * 
         */
        if (debug)
        {
            printf("\nsetpoint values:\n");
        }
        for (unsigned int i = 0; i < NUM_DRIVES; i++) // runs all wheels
        {
            msg.setpoint1 = + wheel_torques[2 * i]     / motor_const; // units: (rad/sec)
            msg.setpoint2 = - wheel_torques[2 * i + 1] / motor_const;

            // msg.setpoint1 = 0.0;
            // msg.setpoint2 = 0.0;
#if SIM_CONTROL == 0
            rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
            *ecData = msg;
#endif
            if (debug)
            {
                printf("%f\t", msg.setpoint1);
                printf("%f\t", msg.setpoint2);
            }
        } // End of the for loop

#if SIM_CONTROL == 0
        /**
         * @brief Construct a new ecx send processdata object
         * 
         */
        ecx_send_processdata(&ecx_context); 

        /**
         * @brief Construct a new ecx receive processdata object
         * 
         */
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        /**
         * @brief receiving updated pivot angles
         * 
         */
        for (unsigned int i = 0; i < NUM_DRIVES; i++)
        {
            txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
            pivot_angles[i] = ecData->encoder_pivot;
        }
#else
        // pivot_angles[0] = 0.0;
        // pivot_angles[1] = 1.0;
        // pivot_angles[2] = -0.5;
        // pivot_angles[3] = 0.3;
        // pivot_angles[0] = 0.0;
        // pivot_angles[1] = 0.0;
        // pivot_angles[2] = 0.0;
        // pivot_angles[3] = 0.0;
#endif
        loop_count += 1;
    } // End of the while loop

    /**
     * @brief releasing memory from all initialised pointers
     * 
     */
    gsl_matrix_free(b);
    gsl_matrix_free(b_verify);

    gsl_matrix_free(A);
    gsl_matrix_free(A_inv_T);
    gsl_matrix_free(A_tmp);
    gsl_matrix_free(A_inv_T_tmp);
    gsl_matrix_free(W);
    gsl_matrix_free(K);
    gsl_vector_free(u);
    gsl_matrix_free(u_inv);
    gsl_matrix_free(V);
    gsl_vector_free(work);

    return 0;
}
