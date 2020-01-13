#!/usr/bin/env python

import sqlite3
import sys

if len(sys.argv) < 2:
    print("Usage: scrubber_db_generator.py databse_name")
    sys.exit()

db_name = sys.argv[1] if sys.argv[1][-3:] == ".db" else sys.argv[1] + ".db"

db = sqlite3.connect("../db/"+db_name)
print("Created %s" % db_name)

c = db.cursor()

c.execute('''CREATE TABLE IF NOT EXISTS `Plan` (
	`plan_id`	INTEGER PRIMARY KEY AUTOINCREMENT UNIQUE,
	`map_id`	INTEGER,
	`zone_id`	INTEGER,
	`name`	text NOT NULL UNIQUE,
	`create_time`	REAL,
	`type`	INTEGER DEFAULT 0,
	`old`	INTEGER DEFAULT 0,
	FOREIGN KEY(`map_id`) REFERENCES `Map`(`map_id`)
);''')

print("Created table Plan")

c.execute('''CREATE TABLE IF NOT EXISTS `ConfigValue` (
    `value_id`	integer,
    `description`	text,
    `value`	integer,
    PRIMARY KEY(`value_id`)
);''')

print("Created table ConfigValue")

c.execute('''CREATE TABLE IF NOT EXISTS `ScrubberConfig` (
    `config_id`	INTEGER UNIQUE,
    `name`	TEXT UNIQUE,
    `squeegee`	INTEGER,
    `brush`	INTEGER,
    `flow`	INTEGER,
    `vacuum`	INTEGER,
    FOREIGN KEY(`flow`) REFERENCES `ConfigValue`(`value_id`),
    FOREIGN KEY(`vacuum`) REFERENCES `ConfigValue`(`value_id`),
    FOREIGN KEY(`brush`) REFERENCES `ConfigValue`(`value_id`),
    PRIMARY KEY(`config_id`)
);''')

print("Created table ScrubberConfig")

c.execute('''CREATE TABLE IF NOT EXISTS `Path` (
	`path_id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT UNIQUE,
	`name`	TEXT NOT NULL UNIQUE,
	`map_id`	INTEGER,
	`type`	INTEGER,
	`old`	INTEGER DEFAULT 0,
	FOREIGN KEY(`map_id`) REFERENCES `Map`(`map_id`)
);''')

print("Created table Path")

c.execute('''CREATE TABLE IF NOT EXISTS `PathData` (
	`path_id`	INTEGER NOT NULL,
	`idx`	INTEGER NOT NULL,
	`x`	REAL NOT NULL,
	`y`	REAL NOT NULL,
	`theta`	REAL NOT NULL,
	PRIMARY KEY(`path_id`,`idx`),
	FOREIGN KEY(`path_id`) REFERENCES `Path`(`path_id`)
);''')

print("Created table PathData")

c.execute('''CREATE TABLE IF NOT EXISTS `Task` (
    `task_id`	INTEGER PRIMARY KEY AUTOINCREMENT UNIQUE,
    `plan_id`	INTEGER,
    `config_id`	INTEGER,
    `type`	INTEGER,
    `idx`	INTEGER,
    FOREIGN KEY(`plan_id`) REFERENCES `Plan`(`plan_id`),
    FOREIGN KEY(`config_id`) REFERENCES `ScrubberConfig`(`config_id`)
);''')

print("Created table Task")

c.execute('''CREATE TABLE IF NOT EXISTS `FollowPath` (
    `task_id`	INTEGER NOT NULL UNIQUE,
    `path_id`	INTEGER NOT NULL,
    FOREIGN KEY(`path_id`) REFERENCES `Path`(`path_id`),
    FOREIGN KEY(`task_id`) REFERENCES `Task`(`task_id`),
    PRIMARY KEY(`task_id`,`path_id`)
);''')

print("Created table FollowPath")

c.execute('''CREATE TABLE IF NOT EXISTS `Map` (
	`map_id`	INTEGER PRIMARY KEY AUTOINCREMENT,
	`name`	TEXT NOT NULL UNIQUE,
	`create_time`	REAL NOT NULL,
	`old`	INTEGER DEFAULT 0
);''')

print("Created table Map")

c.execute('''CREATE TABLE IF NOT EXISTS `MapData` (
	`map_id`	INTEGER,
	`modify_time`	REAL NOT NULL,
	`resolution`	REAL,
	`width`	INTEGER NOT NULL,
	`height`	TEXT NOT NULL,
	`left`	INTEGER,
	`top`	INTEGER,
	`data`	BLOB,
	`original_data`	BLOB,
	`pbstream`	BLOB,
	PRIMARY KEY(`map_id`),
	FOREIGN KEY(`map_id`) REFERENCES `Map`(`map_id`)
);''')

print("Created table MapData")

c.execute('''CREATE TABLE IF NOT EXISTS `Zone` (
	`zone_id`	INTEGER PRIMARY KEY AUTOINCREMENT,
	`name`	INTEGER NOT NULL UNIQUE,
	`map_id`	INTEGER,
	`path_id`	INTEGER,
	`type`	INTEGER,
	`old`	INTEGER DEFAULT 0,
	FOREIGN KEY(`map_id`) REFERENCES `Map`(`map_id`),
	FOREIGN KEY(`path_id`) REFERENCES `Path`(`path_id`)
);''')

print("Created table Zone")

c.execute('''CREATE TABLE IF NOT EXISTS `ZoneConfig` (
    `zone_id`	INTEGER,
    `config_id`	INTEGER,
    FOREIGN KEY(`zone_id`) REFERENCES `Zone`(`zone_id`),
    PRIMARY KEY(`zone_id`,`config_id`),
    FOREIGN KEY(`config_id`) REFERENCES `ScrubberConfig`(`config_id`)
);''')

c.execute('''CREATE TABLE IF NOT EXISTS `Autofill` (
	`zone_id`	INTEGER NOT NULL,
	`plan_id`	INTEGER NOT NULL,
	PRIMARY KEY(`zone_id`,`plan_id`),
	FOREIGN KEY(`zone_id`) REFERENCES `Zone`(`zone_id`),
	FOREIGN KEY(`plan_id`) REFERENCES `Plan`(`plan_id`)
);''')

print("Created table ZoneConfig")

c.execute('''CREATE TABLE IF NOT EXISTS `CleanRecord` (
	`record_id`	INTEGER PRIMARY KEY AUTOINCREMENT UNIQUE,
	`name`	TEXT,
	`map_id`	INTEGER,
	`path_id`	INTEGER,
	`config_id`	INTEGER,
	`area`	INTEGER,
	`duration`	INTEGER,
	`water_comsumption`	INTEGER,
	`power_comsumption`	INTEGER,
	`cover_rate`	REAL,
	`create_time`	INTEGER
);''')

print("Created table CleanRecord")

print("Adding initial datas...")

c.execute('''INSERT INTO ConfigValue VALUES(1, 'OFF', 0);''')
c.execute('''INSERT INTO ConfigValue VALUES(2, 'LOW', 30);''')
c.execute('''INSERT INTO ConfigValue VALUES(3, 'MEDIUM', 60);''')
c.execute('''INSERT INTO ConfigValue VALUES(4, 'HIGH', 80);''')
c.execute('''INSERT INTO ConfigValue VALUES(5, 'SPEED_LOW', 40);''')
c.execute('''INSERT INTO ConfigValue VALUES(6, 'SPEED_MEDIUM', 80);''')
c.execute('''INSERT INTO ConfigValue VALUES(7, 'SPEED_HIGH', 120);''')
c.execute('''INSERT INTO ScrubberConfig VALUES(1, 'OFF_MODE', 0, 1, 1, 1);''')
c.execute('''INSERT INTO ScrubberConfig VALUES(2, 'VACUUM_MODE', 0, 3, 1, 4);''')
c.execute('''INSERT INTO ScrubberConfig VALUES(3, 'ECU_WASH_MODE', 1, 2, 2, 2);''')
c.execute('''INSERT INTO ScrubberConfig VALUES(4, 'STANDARD_MODE', 1, 4, 4, 4);''')
c.execute('''INSERT INTO ScrubberConfig VALUES(5, 'CUSTOM_MODE', 0, 1, 1, 1);''')

db.commit()
db.close()
print("Successfully generated database")

