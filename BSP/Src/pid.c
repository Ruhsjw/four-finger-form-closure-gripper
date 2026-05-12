/*  Floating point PID control loop for Microcontrollers
    Copyright (C) 2015 Jesus Ruben Santa Anna Zamudio.
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    Author website: http://www.geekfactory.mx
    Author e-mail: ruben at geekfactory dot mx
 */
#include "pid.h"
#include "pid_support.h"


PIDPtr pid_create(PIDPtr pid, float in, float out, float set, float kp, float ki, float kd)
{
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;
	pid->automode = false;

   // pid_limits(pid, 0, 255);

    pid->tickspersecond = pid_ticks_per_second();

    /* Set default sample time to 100 ms */
    pid->sampletime = 100 * (pid->tickspersecond / 1000);

    pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);

    pid->lasttime = pid_ticks_get() - pid->sampletime;

    return pid;
}

bool pid_need_compute(PIDPtr pid)
{
    /* Check if the PID period has elapsed */
    return(pid_ticks_get() - pid->lasttime >= pid->sampletime) ? true : false;
}


void pid_speed_comput(PIDPtr pid,float target,float current)
{  float sKp=20;
   float sKi=0.00;
   float sKd=5;

    pid->setpoint = target;
	pid->input = current;
	pid->err[1] = pid->err[0];
	pid->err[0] = target-current;

	pid->p_out = sKp * pid->err[0];
	pid->i_out+= sKi * pid->err[0];
	pid->d_out= sKd * (pid->err[0]-pid->err[1]);
	pid->output = pid->p_out + pid->i_out + pid->d_out;
  Limit_min_max(pid->output, -2000,2000);
}


void pid_angle_comput(PIDPtr pid,float target,float current)
{  float aKp=5;
   float aKi=0;
   float aKd=0;

	pid->setpoint = target;
	pid->input=current;
pid->err[1] = pid->err[0];
pid->err[0] = target-current;
if(pid->err[0]>300){
	pid->err[0]-=360.f;}
	else if(pid->err[0]<-300){
		pid->err[0]+=360.f;
}

pid->p_out = aKp * pid->err[0];
pid->i_out+= aKi * pid->err[0];
pid->d_out= aKd * (pid->err[0]-pid->err[1]);
pid->output = pid->p_out + pid->i_out + pid->d_out;


  Limit_min_max(pid->output,-6000,6000);
}

void pid_tune(PIDPtr pid, float kp, float ki, float kd)
{
    /* Check for validity */
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    //Compute sample time in seconds
    float ssec = ((float) pid->sampletime) / ((float) pid->tickspersecond);

    pid->Kp = kp;
    pid->Ki = ki * ssec;
    pid->Kd = kd / ssec;

    if (pid->direction == E_PID_REVERSE) {
        pid->Kp = 0 - pid->Kp;
        pid->Ki = 0 - pid->Ki;
        pid->Kd = 0 - pid->Kd;
    }
}

void pid_sample(PIDPtr pid, uint32_t time)
{
    if (time > 0) {
        float ratio = (float) (time * (pid->tickspersecond / 1000)) / (float) pid->sampletime;
        pid->Ki *= ratio;
        pid->Kd /= ratio;
        pid->sampletime = time * (pid->tickspersecond / 1000);
    }
}

/*void pid_limits(PIDPtr pid, float min, float max)
{
    if (min >= max) return;
    pid->omin = min;
    pid->omax = max;
    //Adjust output to new limits
    if (pid->automode) {
        if (*(pid->output) > pid->omax)
            *(pid->output) = pid->omax;
        else if (*(pid->output) < pid->omin)
            *(pid->output) = pid->omin;

        if (pid->iterm > pid->omax)
            pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin)
            pid->iterm = pid->omin;
    }
}*/

void pid_auto(PIDPtr pid)
{
    /* If going from manual to auto */
    if (!pid->automode) {
        pid->i_out = pid->output;
        pid->d_out = pid->input;
        if (pid->i_out > pid->omax)
            pid->i_out= pid->omax;
        else if (pid->i_out < pid->omin)
            pid->i_out = pid->omin;
        pid->automode = true;
    }
}

void pid_manual(PIDPtr pid)
{
    pid->automode = false;
}

void pid_direction(PIDPtr pid, enum pid_control_directions dir)
{
    if (pid->automode && pid->direction != dir) {
        pid->Kp = (0 - pid->Kp);
        pid->Ki = (0 - pid->Ki);
        pid->Kd = (0 - pid->Kd);
    }
    pid->direction = dir;
}
