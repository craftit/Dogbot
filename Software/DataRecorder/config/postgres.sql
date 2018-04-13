
--create schemas
CREATE SCHEMA IF NOT EXISTS dogbot1;

-- information table for logging meta-data
CREATE TABLE IF NOT EXISTS dogbot1.log_info (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
comments text );

-- position commands sent to joints
CREATE TABLE IF NOT EXISTS dogbot1.position_command (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
value REAL,
torque_limit REAL);

-- joint control messages
CREATE TABLE IF NOT EXISTS dogbot1.joint_demand (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
position REAL,
effort_limit REAL);

-- joint report
CREATE TABLE IF NOT EXISTS dogbot1.joint_report (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
synctime INT, 
position REAL,
velocity REAL,
effort REAL,
reference TEXT,
position_limit BOOLEAN,
torque_limit BOOLEAN,
velocity_limit BOOLEAN,
index_sensor BOOLEAN
);

-- joint states
CREATE TABLE IF NOT EXISTS dogbot1.joint_state (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
position REAL,
velocity REAL,
effort REAL);

-- Controller parameters
CREATE TABLE IF NOT EXISTS dogbot1.parameter_report (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
parameter text,
value text 
);

-- Information about emergency stops
CREATE TABLE IF NOT EXISTS dogbot1.emergency_stop (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
cause text
);

-- Information about errors
CREATE TABLE IF NOT EXISTS dogbot1.error (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
name text,
cause text,
data text
);

-- joint controller states
CREATE TABLE IF NOT EXISTS dogbot1.joint_state_pid (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
set_point REAL,
process_value REAL,
process_value_dot REAL,
error REAL,
time_step REAL,
command REAL,
p REAL,
i REAL,
d REAL,
i_clamp REAL,
antiwindup boolean);

-- joint states
CREATE TABLE IF NOT EXISTS dogbot1.imu (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
orient_x REAL, orient_y REAL, orient_z REAL, orient_w REAL,
vel_x REAL, vel_y REAL, vel_z REAL,
acc_x REAL, acc_y REAL, acc_z REAL);


--set the search path so queries all work OK
SET search_path TO dogbot1, sim1, public;
