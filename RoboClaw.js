/******************************************************************************
*******************************************************************************
*******************************************************************************

    Copyright (C) 2014 Ben Martin

    libferris is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libferris is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libferris.  If not, see <http://www.gnu.org/licenses/>.

    For more details see the COPYING file in the root directory of this
    distribution.

    $Id: RoboClaw.js,v 1.70 2011/07/31 21:30:49 ben Exp $

*******************************************************************************
*******************************************************************************
******************************************************************************/

var async = require('async');
var b = require('bonescript');
var SerialPort = require("serialport").SerialPort;
var serialport = require("serialport");

var CMD_DRIVEFORWARDM1 = 0;
var CMD_DRIVEBACKWARDM1 = 1;
var CMD_SET_MIN_MAIN_VOLTAGE = 2;
var CMD_SET_MAX_MAIN_VOLTAGE = 3;
var CMD_DRIVEFORWARDM2  = 4;
var CMD_DRIVEBACKWARDM2 = 5;
var CMD_DRIVE_M1        = 6;
var CMD_DRIVE_M2        = 7;
var CMD_DRIVE_FORWARD   = 8;
var CMD_DRIVE_BACKWARDS = 9;
var CMD_TURN_RIGHT      = 10;
var CMD_TURN_LEFT       = 11;
var CMD_DRIVE_FORWARD_OR_BACKWARDS = 12;
var CMD_TURN_LEFT_OR_RIGHT         = 13;
var CMD_READ_ENC_QUAD_M1           = 16;
var CMD_READ_ENC_QUAD_M2           = 17;
var CMD_READ_ENC_SPEED_M1          = 18;
var CMD_READ_ENC_SPEED_M2          = 19;
var CMD_RESET_ENC_COUNTS           = 20;
var CMD_GETVERSION                 = 21;
var CMD_READ_MAIN_BATTERY_VOLTAGE  = 24;
var CMD_READ_LOGIC_BATTERY_VOLTAGE = 25;
var CMD_SET_MIN_LOGIC_VOLTAGE_LEVEL = 26;
var CMD_SET_MAX_LOGIC_VOLTAGE_LEVEL = 27;
var CMD_SET_PID_M1   = 28;
var CMD_SET_PID_M2   = 29;
// var CMD_READ_PID_M1  = 30;
// var CMD_READ_PID_M2  = 31;
var CMD_PID_DRIVE_M1 = 35;
var CMD_PID_DRIVE_M2 = 36;
var CMD_READ_MOTOR_CURRENTS         = 49;
var CMD_READ_MOTOR1_PID_AND_QPPS    = 55;
var CMD_READ_MOTOR2_PID_AND_QPPS    = 56;
var CMD_SET_MAIN_BATTERY_VOLTAGES   = 57;
var CMD_SET_LOGIC_BATTERY_VOLTAGES   = 58;
var CMD_READ_MAIN_BATTERY_VOLTAGES   = 59;
var CMD_READ_LOGIC_BATTERY_VOLTAGES   = 60;
// var CMD_READ_M1_PID   = 63;
// var CMD_READ_M2_PID   = 64;
var CMD_READ_TEMPERATURE = 82;
var CMD_READ_ERROR_STATUS = 90;
var CMD_READ_ENCODER_MODE = 91;
var CMD_SET_M1_ENCODER_MODE = 92;
var CMD_SET_M2_ENCODER_MODE = 93;
var CMD_WRITE_SETTINGS_TO_EEPROM = 94;

// up to encoders on page 47.



function CheckedWriter( uartport )
{
    this.uartport = uartport;
}
CheckedWriter.prototype.write = function(x) {
}

function emptyCallback( x )
{
    console.log("emptyCallback x.data: " + x.data);
}

function RoboClaw( uartport, roboclawaddr, cfgbaudrate, callback )
{
    this.uartport = uartport;
    this.roboclawaddr = typeof roboclawaddr !== 'undefined' ? roboclawaddr : 0x80;
    this.baudrate     = typeof  cfgbaudrate !== 'undefined' ? cfgbaudrate : 38400;
    this.dataCallback = emptyCallback;
    this.robomethid   = 0;
    this.runningTask  = 0;
    this.carryData = new Buffer(0);

    console.log("setting up for port:" + this.uartport );
    var options = {
	baudrate: this.baudrate
    };
    var self = this;
    var sp = new SerialPort(this.uartport, {
	baudrate: 38400
//	, bufferSize: 1
//	, parser: serialport.parsers.raw
    });
    this.sp = sp;

    this.q = async.queue(function (task, callback) {
	console.log('roboclaw queue: ' + task.name);
	task.func();
	callback();
    }, 1 );
    this.q.drain = function() {
	console.log('roboclaw queue: all items have been processed');
    }
    self.tasks = [];

    sp.on("data", function (data) {
	console.log("serial.on(DATA) rmeth:" + self.robomethid ); // + " _data: " + data);

	var cbdata = { rc: self, data: data };
	if( self.robomethid == CMD_GETVERSION )
	{
	    var b = self.onData_getRunningBuffer( data );
	    var i = 0;
	    for( i=0; i<b.length; i++ )
	    {
		if( b[i] == 0 )
		{
		    if( i == b.length-1 )
		    {
			return;
		    }
		}
	    }
	}
	if( self.robomethid == CMD_READ_TEMPERATURE ) 
	{
	    var b = self.onData_getRunningBuffer( data );
	    console.log("b.len:" + b.length );
	    if( b.length < 2 )
		return;
	    cbdata.temperature = b.readUInt16BE( 0 ) / 10;
	}
	if( self.robomethid == CMD_READ_ERROR_STATUS )
	{
	    var b = self.onData_getRunningBuffer( data );
	    if( b.length < 2 )
		return;
	    console.log("b.len:" + b.length );
	    cbdata.roboerror = b.readUInt8( 0 );
	}
	if( self.robomethid == CMD_READ_ENCODER_MODE )
	{
	    var b = self.onData_getRunningBuffer( data );
	    if( b.length < 3 )
		return;
	    cbdata.m1absolute      = b[0] & (1<<0);
	    cbdata.m1analogSupport = b[0] & (1<<7);
	    cbdata.m2absolute      = b[1] & (1<<0);
	    cbdata.m2analogSupport = b[1] & (1<<7);
	}
	if( self.robomethid == CMD_READ_MAIN_BATTERY_VOLTAGE )
	{
	    var b = self.onData_getRunningBuffer( data );
	    if( b.length < 3 )
		return;
	    cbdata.voltage = b.readUInt16BE( 0 ) / 10;
	}
	if( self.robomethid == CMD_READ_LOGIC_BATTERY_VOLTAGE )
	{
	    var b = self.onData_getRunningBuffer( data );
	    console.log("25, b.len:" + b.length );
	    if( b.length < 3 )
		return;
	    cbdata.voltage = b.readUInt16BE( 0 ) / 10;
	}
	if( self.robomethid == CMD_READ_MAIN_BATTERY_VOLTAGES )
	{
	    var b = self.onData_getRunningBuffer( data );
	    console.log("b.len:" + b.length );
	    if( b.length < 5 )
		return;
	    cbdata.vmin = b.readUInt16BE( 0 ) / 10;
	    cbdata.vmax = b.readUInt16BE( 2 ) / 10;
	}
	if( self.robomethid == CMD_READ_ENC_QUAD_M1 
	    || self.robomethid == CMD_READ_ENC_QUAD_M2 )
	{
	    var b = self.onData_getRunningBuffer( data );
	    console.log("CMD_READ_ENC_QUAD b.len:" + b.length );
	    if( b.length < 6 )
		return;
 	    cbdata.count  = b.readUInt32BE( 0 );
 	    cbdata.value  = cbdata.count;
	    cbdata.status = b[4];
	    cbdata.forwards = (cbdata.status & (1<<1)) == 0;
	    cbdata.backwars = !cbdata.forwards;
	    cbdata.underflow = (cbdata.status & (1<<0));
	    cbdata.overflow  = (cbdata.status & (1<<2));
	}
	if( self.robomethid == CMD_READ_ENC_SPEED_M1
	    || self.robomethid == CMD_READ_ENC_SPEED_M2 )
	{
	    var b = self.onData_getRunningBuffer( data );
	    console.log("b.len:" + b.length );
	    if( b.length < 6 )
		return;
	    cbdata.count  = b.readUInt32BE( 0 );
	    cbdata.forwards = b[4] > 0;
	    cbdata.backwars = !cbdata.forwards;
	}

	self.dataCallback( cbdata );
	self.runningTask = 0;
	self.dataCallback = emptyCallback;
	self.robomethid = 0;
	self.carryData = new Buffer(0);

	self.tryRunNextTask();

	// FIXME: for 24 need to parse 2 bytes to a single int value

	// FIXME: for CMD_READ_MOTOR_CURRENTS
	//         need to combine 2 16 bit values.

	// FIXME: for CMD_READ_MOTOR1_PID_AND_QPPS need to break out 4 values.


    });
    sp.on("open", function () {
	console.log('serial port is now open');
// 	self.getVersion();
	callback( { rc: self, open: 1 } );
    });
    sp.on("error", function (data) {
	console.log("error: "+data);
	callback( { rc: self, error: data } );
    });
//    b.serialOpen( this.uartport, options, function(x) { self.onSerial(x); });



}

RoboClaw.prototype.onData_getRunningBuffer = function( data ) 
{
    var ret = new Buffer( this.carryData.toString() + data );
    this.carryData = ret;
    return ret;
}


RoboClaw.prototype.tryRunNextTask = function() 
{
    var self = this;

    // self.tasks.push( { b: b, callback: callback, robomethid: self.robomethid  } );
    console.log("tryRunNextTask() tasks.len:" + self.tasks.length + " runningtask:" + self.runningTask );
    if( self.tasks.length && !self.runningTask )
    {
	var t = self.tasks.shift();
	console.log("STARTING NEXT TASK 1... rmeth:" + t.robomethid );
	if( t.robomethid == -1 )
	{
	    console.log("task is just a write, will try to run next task as well" );
	    this.sp.write( t.b, function() {
		self.tryRunNextTask();
	    });
	    return;
	}
	this.robomethid = t.robomethid;
	this.dataCallback = t.callback;
	self.runningTask = 1;
	this.sp.write( t.b );
	console.log("STARTING NEXT TASK 2...");
    }
}

RoboClaw.prototype.dispatchMethodReturnsData = function( buf, callback ) 
{
    this.dataCallback = callback;
    this.sp.write( b );
}

RoboClaw.prototype.dispatchMethod2bReturnsData = function( b1, b2, callback ) 
{
    var robomethid = b2;
    var b = new Buffer(2);
    b[0] = b1;
    b[1] = b2;

//    this.robomethid = robomethid;
//    this.dataCallback = callback;
//    this.sp.write( b );

    console.log("dispatchMethod2bReturnsData()");
    var self = this;
    self.tasks.push( { b: b, callback: callback, robomethid: robomethid  } );
    this.tryRunNextTask();
}


/**
 * Write databytes from the data buffer with the checksum appended.
 */
RoboClaw.prototype.writeAddChecksum = function( data, databytes ) 
{
    var checksum = 0;
    var i = 0;
    var b = new Buffer(databytes+1);
    for( i=0; i<databytes; i++ )
    {
	b[i] = data[i];
	checksum += b[i];
    }
    checksum = checksum & 0x7f;
    b[databytes] = checksum;
//    this.sp.write( b );

    console.log("writeAddChecksum()");
    var self = this;
    self.tasks.push( { b: b, callback: emptyCallback, robomethid: -1  } );
    self.tryRunNextTask();

}
RoboClaw.prototype.write1AddChecksum = function( cmd ) 
{
    var bidx = 0;
    var b = new Buffer(30);
    b[bidx++] = this.roboclawaddr;
    b[bidx++] = cmd;
    this.writeAddChecksum( b, bidx );
}
RoboClaw.prototype.write2AddChecksum = function( cmd, b1 ) 
{
    var bidx = 0;
    var b = new Buffer(30);
    b[bidx++] = this.roboclawaddr;
    b[bidx++] = cmd;
    b[bidx++] = b1;
    this.writeAddChecksum( b, bidx );
}

RoboClaw.prototype.setMotorSpeed = function( motor, moveForward, speed ) 
{
    var cmd = 0;
    cmd = (motor == 1) ? CMD_DRIVEFORWARDM1 : CMD_DRIVEFORWARDM2;
    if( !moveForward )
	cmd = (motor == 1) ? CMD_DRIVEBACKWARDM1 : CMD_DRIVEBACKWARDM2;

    console.log("setMotorSpeed() motor:" + motor + " cmd:" + cmd + " speed:" + speed );
    this.write2AddChecksum( cmd, speed );

    // var bidx = 0;
    // var b = new Buffer(4);
    // b[bidx++] = 0x80;
    // b[bidx++] = cmd;
    // b[bidx++] = speed;

    // var i = 0;
    // b[bidx] = 0;
    // for( i=0; i<bidx; i++ )
    // 	b[bidx] += b[i];
    // b[bidx] = b[bidx] & 0x7f;
    // this.sp.write( b );
}

RoboClaw.prototype.stopAll = function() 
{
    this.setMotorSpeed( 1, 1, 0 );
    this.setMotorSpeed( 2, 1, 0 );
}

/****************************************/
/****************************************/
/****************************************/

RoboClaw.prototype.driveForwardM1 = function( speed ) 
{
    console.log("driveForwardM1() speed:" + speed);
    this.setMotorSpeed( 1, 1, speed );
}

RoboClaw.prototype.driveBackwardsM1 = function( speed ) 
{
    this.setMotorSpeed( 1, 0, speed );
}

RoboClaw.prototype.setMinimumMainVoltage = function( volts ) 
{
    if( volts < 6 )
	volts = 6;
    if( volts > 30 )
	volts = 30;

    var value = (volts - 6) * 5;
    this.write2AddChecksum( CMD_SET_MIN_MAIN_VOLTAGE, value );
}

RoboClaw.prototype.setMaximumMainVoltage = function( volts ) 
{
    if( volts < 0 )
	volts = 0;
    if( volts > 30 )
	volts = 30;

    var value = Math.ceil( volts * 5.12 );
    this.write2AddChecksum( CMD_SET_MAX_MAIN_VOLTAGE, value );
}

RoboClaw.prototype.driveForwardM2 = function( speed ) 
{
    this.setMotorSpeed( 2, 1, speed );
}

RoboClaw.prototype.driveBackwardsM2 = function( speed ) 
{
    this.setMotorSpeed( 2, 0, speed );
}

RoboClaw.prototype.clampInt = function( v, vmin, vmax ) 
{
    v = Math.ceil( v );
    if( v < vmin )
	v = vmin;
    if( v > vmax )
	v = vmax;
    return v;
}
RoboClaw.prototype.driveM1 = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_DRIVE_M1, speed );
}

RoboClaw.prototype.driveM2 = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_DRIVE_M2, speed );
}


RoboClaw.prototype.driveForward = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_DRIVE_FORWARD, speed );
}

RoboClaw.prototype.driveBackwards = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_DRIVE_BACKWARDS, speed );
}

RoboClaw.prototype.turnRight = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_TURN_RIGHT, speed );
}

RoboClaw.prototype.turnLeft = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_TURN_LEFT, speed );
}

// 64 = stop
RoboClaw.prototype.drive = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_DRIVE_FORWARD_OR_BACKWARDS, speed );
}

// 64 = stop
RoboClaw.prototype.stop = function() 
{
    this.drive( 64 );
}

// 64 = no turning.
RoboClaw.prototype.turn = function( speed ) 
{
    speed = this.clampInt( speed, 0, 127 );
    this.write2AddChecksum( CMD_TURN_LEFT_OR_RIGHT, speed );
}


RoboClaw.prototype.getVersion = function( callback ) 
{
    console.log("writing to roboclaw...");
    this.dispatchMethod2bReturnsData( this.roboclawaddr, CMD_GETVERSION, callback );
    // var b = new Buffer(2);
    // b[0] = this.roboclawaddr;
    // b[1] = CMD_GETVERSION;
    // this.dispatchMethod( b, callback );
}

RoboClaw.prototype.getMainBatteryVoltage = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_MAIN_BATTERY_VOLTAGE, 
				      callback );
}

RoboClaw.prototype.getLogicBatteryVoltage = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_LOGIC_BATTERY_VOLTAGE, 
				      callback );
}


RoboClaw.prototype.setMinimumLogicVoltageLevel = function( volts ) 
{
    var value = (volts - 6) * 5;
    this.write2AddChecksum( CMD_SET_MIN_LOGIC_VOLTAGE_LEVEL, value );
}

RoboClaw.prototype.setMaximumLogicVoltageLevel = function( volts ) 
{
    var value = Math.ceil( volts * 5.12 );
    this.write2AddChecksum( CMD_SET_MAX_LOGIC_VOLTAGE_LEVEL, value );
}

RoboClaw.prototype.getMotorCurrents = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_MOTOR_CURRENTS, 
				      callback );
}

RoboClaw.prototype.getMotor1PIDandQPPS = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_MOTOR1_PID_AND_QPPS, 
				      callback );
}

RoboClaw.prototype.getMotor2PIDandQPPS = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_MOTOR2_PID_AND_QPPS, 
				      callback );
}


RoboClaw.prototype.setMainBatteryVoltages = function( vmin, vmax ) 
{
    var bidx = 0;
    var b = new Buffer(30);
    b[bidx++] = this.roboclawaddr;
    b[bidx++] = CMD_SET_MAIN_BATTERY_VOLTAGES;
    b.writeUInt16BE( vmin, bidx );
    bidx += 2;
    b.writeUInt16BE( vmax, bidx );
    bidx += 2;
    this.writeAddChecksum( b, bidx );
}

RoboClaw.prototype.setLogicBatteryVoltages = function( vmin, vmax ) 
{
    var bidx = 0;
    var b = new Buffer(30);
    b[bidx++] = this.roboclawaddr;
    b[bidx++] = CMD_SET_LOGIC_BATTERY_VOLTAGES;
    b.writeUInt16BE( vmin, bidx );
    bidx += 2;
    b.writeUInt16BE( vmax, bidx );
    bidx += 2;
    this.writeAddChecksum( b, bidx );
}

RoboClaw.prototype.getMainBatteryVoltages = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_MAIN_BATTERY_VOLTAGES, 
				      callback );
}

RoboClaw.prototype.getLogicBatteryVoltages = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_LOGIC_BATTERY_VOLTAGES, 
				      callback );
}

RoboClaw.prototype.getTemperature = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_TEMPERATURE, 
				      callback );
}

RoboClaw.prototype.getErrorStatus = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_ERROR_STATUS, 
				      callback );
}

RoboClaw.prototype.getEncoderMode = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_ENCODER_MODE, 
				      callback );
}

RoboClaw.prototype.setM1EncoderMode = function( analogSupport, absolute ) 
{
    var value = 0;
    if( analogSupport )
	value |= (1<<7);
    if( absolute )
	value |= (1<<0);
    this.write2AddChecksum( CMD_SET_M1_ENCODER_MODE, value );
}

RoboClaw.prototype.setM2EncoderMode = function( analogSupport, absolute ) 
{
    var value = 0;
    if( analogSupport )
	value |= (1<<7);
    if( absolute )
	value |= (1<<0);
    this.write2AddChecksum( CMD_SET_M2_ENCODER_MODE, value );
}

RoboClaw.prototype.writeSettingsToEEPROM = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_WRITE_SETTINGS_TO_EEPROM, 
				      callback );
}


RoboClaw.prototype.readEncQuadM1 = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_ENC_QUAD_M1, 
				      callback );
}

RoboClaw.prototype.readEncQuadM2 = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_ENC_QUAD_M2, 
				      callback );
}

RoboClaw.prototype.readEncSpeedM1 = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_ENC_SPEED_M1, 
				      callback );
}

RoboClaw.prototype.readEncSpeedM2 = function( callback ) 
{
    this.dispatchMethod2bReturnsData( this.roboclawaddr, 
				      CMD_READ_ENC_SPEED_M2, 
				      callback );
}

RoboClaw.prototype.resetEncCounts = function( callback ) 
{
    this.write1AddChecksum( CMD_RESET_ENC_COUNTS );
}


RoboClaw.prototype.setPIDMx = function( cmd, P, I, D, QPPS ) 
{
    if( typeof P === 'undefined' || P == 0 )
	P = 0x00010000;
    if( typeof I === 'undefined' || I == 0 )
	I = 0x00008000;
    if( typeof D === 'undefined' || D == 0 )
	D = 0x00004000;
    if( typeof QPPS === 'undefined' || QPPS == 0 )
	QPPS = 44000;

    var bidx = 0;
    var b = new Buffer(60);
    b[bidx++] = this.roboclawaddr;
    b[bidx++] = cmd;
    b.writeUInt32BE( D,    bidx );    bidx+=4;
    b.writeUInt32BE( P,    bidx );    bidx+=4;
    b.writeUInt32BE( I,    bidx );    bidx+=4;
    b.writeUInt32BE( QPPS, bidx );    bidx+=4;
    this.writeAddChecksum( b, bidx );
}

RoboClaw.prototype.setPIDM1 = function( P, I, D, QPPS ) 
{
    this.setPIDMx( CMD_SET_PID_M1, P, I, D, QPPS );
}
RoboClaw.prototype.setPIDM2 = function( P, I, D, QPPS ) 
{
    this.setPIDMx( CMD_SET_PID_M2, P, I, D, QPPS );
}

RoboClaw.prototype.setQPPSM1 = function( P, I, D, QPPS ) 
{
    this.setPIDMx( CMD_SET_PID_M1, 0,0,0, QPPS );
}
RoboClaw.prototype.setQPPSM2 = function( P, I, D, QPPS ) 
{
    this.setPIDMx( CMD_SET_PID_M2, 0,0,0, QPPS );
}


RoboClaw.prototype.setPIDDriveMx = function( cmd, qspeed ) 
{
    if( typeof qspeed === 'undefined' || qspeed == 0 )
	qspeed = 0;
    if( !qspeed )
    {
	return;
    }

    var bidx = 0;
    var b = new Buffer(60);
    b[bidx++] = this.roboclawaddr;
    b[bidx++] = cmd;
    b.writeUInt32BE( qspeed, bidx ); 
    bidx+=4;
    this.writeAddChecksum( b, bidx );
}

RoboClaw.prototype.setPIDDriveM1 = function( qspeed ) 
{
    this.setPIDDriveMx( CMD_PID_DRIVE_M1, qspeed );
}
RoboClaw.prototype.setPIDDriveM2 = function( qspeed ) 
{
    this.setPIDDriveMx( CMD_PID_DRIVE_M2, qspeed );
}



//RoboClaw.prototype.getVersionb = function() {
//    b.serialWrite( this.uartport, this.roboclawaddr );
//    b.serialWrite( this.uartport, CMD_GETVERSION );
//}

// RoboClaw.prototype.onSerial = function(x) {

//     if (x.err) {
//         console.log('***ERROR*** ' + JSON.stringify(x));
//     }
//     if (x.event == 'open') {
// 	console.log('***OPENED***');
// 	this.getVersion();
//     }
//     if (x.event == 'data') {
//         console.log('read from uart:' + String(x.data));
//     }
// }


RoboClaw.errorHandler = function (error) 
{
    console.log('Error: ' + error.message);
};
module.exports = RoboClaw;
