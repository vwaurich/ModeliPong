within ;
package ModeliPong
  model Game

    parameter Real fieldHeight = 1;
    parameter Real fieldWidth = 2;
    parameter Real barLength = 0.5;
    parameter Real ballRad = 0.1;
    constant Real width = 0.05;

    parameter Real phi0 = 90*Modelica.Constants.pi/360;
    parameter Real vel0 = 0.5;

    Real p1, p2;
    Real ballX,ballY;
    Real vel( start = vel0);
    discrete Real phi( start = phi0);
    Real phi_deg = phi/Modelica.Constants.pi*360;

    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape boundary_top(
      length=fieldWidth,
      width=width,
      height=width,
      color={0,0,0},
      r={-fieldWidth/2,fieldHeight/2 + barLength/2,0})             annotation (Placement(transformation(extent={{-20,40},
              {0,60}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape boundary_bottom(
      length=fieldWidth,
      width=width,
      height=width,
      color={0,0,0},
      r={-fieldWidth/2,-fieldHeight/2 - barLength/2,0})
      annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape player1Bar(
      length=width,
      width=barLength,
      height=width,
      color={0,0,255},
      r={-fieldWidth/2,p1,0})
      annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape player2Bar(
      length=width,
      width=barLength,
      height=width,
      r={fieldWidth/2,p2,0},
      color={0,127,0})
                  annotation (Placement(transformation(extent={{20,0},{40,20}})));

    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape shape(
      shapeType="sphere",
      color={255,128,0},
      length=ballRad,
      width=ballRad,
      height=ballRad,
      r={ballX,ballY,0})
                      annotation (Placement(transformation(extent={{-20,0},{0,20}})));
    Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialReceive(
      baud=Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B9600,
      parity=0,
      enableExternalTrigger=false,
      startTime=0.0,
      autoBufferSize=false,
      sampleTime=0.2,
      Serial_Port="COM3",
      userBufferSize=4)
      annotation (Placement(transformation(extent={{-180,60},{-160,80}})));
    Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger
                                                                                 unpackInt(bitOffset=
         0, width=16,
      nu=1)
      annotation (Placement(transformation(extent={{-160,40},{-140,60}})));
    Modelica.Blocks.Math.IntegerToReal integerToReal
      annotation (Placement(transformation(extent={{-120,40},{-100,60}})));
    Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger
      unpackInt1(width=16)
      annotation (Placement(transformation(extent={{-160,0},{-140,20}})));
    Modelica.Blocks.Math.IntegerToReal integerToReal1
      annotation (Placement(transformation(extent={{-120,0},{-100,20}})));
  /*
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime
    synchronizeRealtime
    annotation (Placement(transformation(extent={{-140,-40},{-120,-20}})));
*/
  equation
     p1 = ((integerToReal.y/1032)-0.5)*fieldHeight;
     p2 = ((integerToReal1.y/1032)-0.5)*fieldHeight;
     der(ballY) = sin(phi)*vel;
     der(ballX) = cos(phi)*vel;
     //increase velocity
     vel = vel0+time*0.01;

     //hit
     when ballY >= (fieldHeight/2+barLength/2) then
       phi = -pre(phi);
     elsewhen ballY <= (-fieldHeight/2-barLength/2) then
       phi = -pre(phi);
     elsewhen abs(ballX) > (fieldWidth/2) then
       phi = -pre(phi) + Modelica.Constants.pi;
     end when;

     when abs(ballY - p1) > barLength/2 and ballX <= -(fieldWidth/2) then
       reinit(ballX, 0);
       reinit(ballY, 0);
     elsewhen abs(ballY - p2) > barLength/2 and ballX >= (fieldWidth/2) then
       reinit(ballX, 0);
       reinit(ballY, 0);
     end when;

    connect(serialReceive.pkgOut,unpackInt. pkgIn) annotation (Line(
        points={{-159.2,70},{-150,70},{-150,60.8}}));
    connect(unpackInt.y,integerToReal. u) annotation (Line(
        points={{-139,50},{-122,50}},
        color={255,127,0},
        smooth=Smooth.None));
    connect(unpackInt.pkgOut[1],unpackInt1. pkgIn) annotation (Line(
        points={{-150,39.2},{-150,20.8}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(unpackInt1.y,integerToReal1. u) annotation (Line(
        points={{-139,10},{-122,10}},
        color={255,127,0},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(extent={{-180,-100},{100,100}})), Icon(
          coordinateSystem(extent={{-180,-100},{100,100}})));
  end Game;
  annotation (uses(Modelica(version="3.2.1"), Modelica_DeviceDrivers(version=
            "1.4.4")));
end ModeliPong;
