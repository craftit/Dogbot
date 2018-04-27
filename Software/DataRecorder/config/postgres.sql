
-- Create table of devices
CREATE TABLE IF NOT EXISTS source (id BIGSERIAL PRIMARY KEY, 
sourceid text  UNIQUE,
devicetype text,
serialnumber text,
notes text
);

-- information table for logging meta-data
CREATE TABLE IF NOT EXISTS log_info (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
comments text );

-- position commands sent to joints
CREATE TABLE IF NOT EXISTS position_command (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
value REAL,
torque_limit REAL);

-- joint control messages
CREATE TABLE IF NOT EXISTS joint_demand (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
position REAL,
effort_limit REAL);


-- joint report
CREATE TABLE IF NOT EXISTS joint_report (id BIGSERIAL PRIMARY KEY, 
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

CREATE INDEX IF NOT EXISTS joint_report_sourceid_logtime_idx ON joint_report (sourceid,logtime);

-- joint states
CREATE TABLE IF NOT EXISTS joint_state (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
position REAL,
velocity REAL,
effort REAL);

-- Controller parameters
CREATE TABLE IF NOT EXISTS parameter_report (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
parameter text,
value text 
);

CREATE INDEX IF NOT EXISTS parameter_report_sourceid_parameter_logtime_idx ON parameter_report (sourceid,parameter,logtime);

-- Information about emergency stops
CREATE TABLE IF NOT EXISTS emergency_stop (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
cause text
);

CREATE INDEX IF NOT EXISTS emergency_stop_sourceid_logtime_idx ON emergency_stop (sourceid,logtime);

-- Information about errors
CREATE TABLE IF NOT EXISTS error (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(),
name text,
cause text,
data text
);

CREATE INDEX IF NOT EXISTS error_sourceid_logtime_idx ON error (sourceid,logtime);

-- joint controller states
CREATE TABLE IF NOT EXISTS joint_state_pid (id BIGSERIAL PRIMARY KEY, 
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
CREATE TABLE IF NOT EXISTS imu (id BIGSERIAL PRIMARY KEY, 
sourceid text,
logtime TIMESTAMPTZ DEFAULT now(), 
orient_x REAL, orient_y REAL, orient_z REAL, orient_w REAL,
vel_x REAL, vel_y REAL, vel_z REAL,
acc_x REAL, acc_y REAL, acc_z REAL);


--set the search path so queries all work OK
--SET search_path TO dogbot1, sim1, public;
