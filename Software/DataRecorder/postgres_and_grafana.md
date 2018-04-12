Grafana and dogbot logging:

# Running grafana

sudo service grafana-server start
go to http://127.0.0.1:3000 in browser
default admin user is 'admin'

# Install

http://docs.grafana.org/installation/debian/   -follow apt install

summary of where stuff ends up:
    Installs binary to /usr/sbin/grafana-server
    Installs Init.d script to /etc/init.d/grafana-server
    Creates default file (environment vars) to /etc/default/grafana-server
    Installs configuration file to /etc/grafana/grafana.ini
    Installs systemd service (if systemd is available) name grafana-server.service
    The default configuration sets the log file at /var/log/grafana/grafana.log
    The default configuration specifies an sqlite3 db at /var/lib/grafana/grafana.db
    Installs HTML/JS/CSS and other Grafana files at /usr/share/grafana
    
# Setting up postgresql

    psql -U reactai
    CREATE DATABASE dogbot;

then back at a command prompt, this should work:

    psql -U reactai dogbot <./config/postgres.sql 

when using psql:
    \c dogbot;
    SET search_path TO dogbot1, sim1, public;
    

# logging to postgres
 
from the simulation, in two windows
roslaunch dogbot gazebo.launch
roslaunch dogbot logger.launch logging_config:=sim_logging.yaml