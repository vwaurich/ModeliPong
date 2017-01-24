package ModeliPong
  model Game_keyboard
    Game game(vel0 = 1) annotation(
      Placement(transformation(extent = {{50, -20}, {78, 0}})));
    Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime annotation(
      Placement(transformation(extent = {{-160, 62}, {-140, 82}})));
    Modelica_DeviceDrivers.Blocks.InputDevices.KeyboardKeyInput player1_up(keyCode = "Q") annotation(
      Placement(transformation(extent = {{-160, 20}, {-140, 40}})));
    Modelica_DeviceDrivers.Blocks.InputDevices.KeyboardKeyInput player1_down(keyCode = "A") annotation(
      Placement(transformation(extent = {{-160, -6}, {-140, 14}})));
    Modelica_DeviceDrivers.Blocks.InputDevices.KeyboardKeyInput player2_up(keyCode = "O") annotation(
      Placement(transformation(extent = {{-160, -40}, {-140, -20}})));
    Modelica_DeviceDrivers.Blocks.InputDevices.KeyboardKeyInput player2_down(keyCode = "L") annotation(
      Placement(transformation(extent = {{-160, -66}, {-140, -46}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal(realTrue = 1) annotation(
      Placement(transformation(extent = {{-120, 20}, {-100, 40}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1(realTrue = -1) annotation(
      Placement(transformation(extent = {{-120, -6}, {-100, 14}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal2(realTrue = 1) annotation(
      Placement(transformation(extent = {{-120, -40}, {-100, -20}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal3(realTrue = -1) annotation(
      Placement(transformation(extent = {{-120, -66}, {-100, -46}})));
    Modelica.Blocks.Continuous.Integrator integrator annotation(
      Placement(transformation(extent = {{-80, 20}, {-60, 40}})));
    Modelica.Blocks.Continuous.Integrator integrator1 annotation(
      Placement(transformation(extent = {{-80, -6}, {-60, 14}})));
    Modelica.Blocks.Continuous.Integrator integrator2 annotation(
      Placement(transformation(extent = {{-80, -40}, {-60, -20}})));
    Modelica.Blocks.Continuous.Integrator integrator3 annotation(
      Placement(transformation(extent = {{-80, -66}, {-60, -46}})));
    Modelica.Blocks.Math.Add add annotation(
      Placement(transformation(extent = {{-42, 8}, {-22, 28}})));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(transformation(extent = {{-40, -54}, {-20, -34}})));
    Modelica.Blocks.Math.Min min annotation(
      Placement(transformation(extent = {{0, 16}, {10, 26}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y = 1) annotation(
      Placement(transformation(extent = {{-40, 46}, {-20, 66}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y = 0) annotation(
      Placement(transformation(extent = {{-40, -86}, {-20, -66}})));
    Modelica.Blocks.Math.Max max2 annotation(
      Placement(transformation(extent = {{18, 16}, {30, 26}})));
    Modelica.Blocks.Math.Min min1 annotation(
      Placement(transformation(extent = {{-2, -52}, {8, -42}})));
    Modelica.Blocks.Math.Max max1 annotation(
      Placement(transformation(extent = {{18, -52}, {30, -42}})));
  equation
    connect(player1_up.keyState, booleanToReal.u) annotation(
      Line(points = {{-139, 30}, {-122, 30}}, color = {255, 0, 255}));
    connect(booleanToReal1.u, player1_down.keyState) annotation(
      Line(points = {{-122, 4}, {-139, 4}}, color = {255, 0, 255}));
    connect(booleanToReal2.u, player2_up.keyState) annotation(
      Line(points = {{-122, -30}, {-139, -30}}, color = {255, 0, 255}));
    connect(booleanToReal3.u, player2_down.keyState) annotation(
      Line(points = {{-122, -56}, {-139, -56}}, color = {255, 0, 255}));
    connect(integrator.u, booleanToReal.y) annotation(
      Line(points = {{-82, 30}, {-99, 30}}, color = {0, 0, 127}));
    connect(integrator1.u, booleanToReal1.y) annotation(
      Line(points = {{-82, 4}, {-82, 4}, {-99, 4}}, color = {0, 0, 127}));
    connect(booleanToReal2.y, integrator2.u) annotation(
      Line(points = {{-99, -30}, {-82, -30}}, color = {0, 0, 127}));
    connect(integrator3.u, booleanToReal3.y) annotation(
      Line(points = {{-82, -56}, {-99, -56}}, color = {0, 0, 127}));
    connect(realExpression.y, min.u1) annotation(
      Line(points = {{-19, 56}, {-4, 56}, {-4, 24}, {-1, 24}}, color = {0, 0, 127}));
    connect(add.u1, integrator.y) annotation(
      Line(points = {{-44, 24}, {-54, 24}, {-54, 30}, {-59, 30}}, color = {0, 0, 127}));
    connect(add.u2, integrator1.y) annotation(
      Line(points = {{-44, 12}, {-54, 12}, {-54, 4}, {-59, 4}}, color = {0, 0, 127}));
    connect(add1.u1, integrator2.y) annotation(
      Line(points = {{-42, -38}, {-56, -38}, {-56, -30}, {-59, -30}}, color = {0, 0, 127}));
    connect(add1.u2, integrator3.y) annotation(
      Line(points = {{-42, -50}, {-56, -50}, {-56, -56}, {-59, -56}}, color = {0, 0, 127}));
    connect(add.y, min.u2) annotation(
      Line(points = {{-21, 18}, {-12, 18}, {-1, 18}}, color = {0, 0, 127}));
    connect(min.y, max2.u1) annotation(
      Line(points = {{10.5, 21}, {14, 21}, {14, 24}, {16.8, 24}}, color = {0, 0, 127}));
    connect(min1.y, max1.u1) annotation(
      Line(points = {{8.5, -47}, {12, -47}, {12, -44}, {16.8, -44}}, color = {0, 0, 127}));
    connect(add1.y, min1.u1) annotation(
      Line(points = {{-19, -44}, {-3, -44}, {-3, -44}}, color = {0, 0, 127}));
    connect(min1.u2, min.u1) annotation(
      Line(points = {{-3, -50}, {-8, -50}, {-8, 56}, {-4, 56}, {-4, 24}, {-1, 24}}, color = {0, 0, 127}));
    connect(realExpression1.y, max1.u2) annotation(
      Line(points = {{-19, -76}, {10, -76}, {10, -50}, {16.8, -50}}, color = {0, 0, 127}));
    connect(max2.u2, max1.u2) annotation(
      Line(points = {{16.8, 18}, {14, 18}, {14, 16}, {10, 16}, {10, -50}, {16.8, -50}}, color = {0, 0, 127}));
    connect(max1.y, game.player2) annotation(
      Line(points = {{30.6, -47}, {40, -47}, {40, -15}, {50, -15}}, color = {0, 0, 127}));
    connect(max2.y, game.player1) annotation(
      Line(points = {{30.6, 21}, {40, 21}, {40, -3}, {50, -3}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-180, -100}, {100, 100}}, preserveAspectRatio = false)),
      Icon(coordinateSystem(extent = {{-180, -100}, {100, 100}})));
  end Game_keyboard;

  model Game_arduino
    Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialReceive(baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B9600, parity = 0, enableExternalTrigger = false, startTime = 0.0, autoBufferSize = false, sampleTime = 0.2, Serial_Port = "COM3", userBufferSize = 4) annotation(
      Placement(transformation(extent = {{-180, 60}, {-160, 80}})));
    Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger unpackInt(bitOffset = 0, width = 16, nu = 1) annotation(
      Placement(transformation(extent = {{-160, 40}, {-140, 60}})));
    Modelica.Blocks.Math.IntegerToReal integerToReal annotation(
      Placement(transformation(extent = {{-120, 40}, {-100, 60}})));
    Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger unpackInt1(width = 16) annotation(
      Placement(transformation(extent = {{-160, 0}, {-140, 20}})));
    Modelica.Blocks.Math.IntegerToReal integerToReal1 annotation(
      Placement(transformation(extent = {{-120, 0}, {-100, 20}})));
    /*
          Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime
            synchronizeRealtime
            annotation (Placement(transformation(extent={{-140,-40},{-120,-20}})));
        */
    Game game annotation(
      Placement(transformation(extent = {{4, 18}, {32, 38}})));
    Modelica.Blocks.Math.Gain gain(k = 1 / 1032) annotation(
      Placement(transformation(extent = {{-80, 40}, {-60, 60}})));
    Modelica.Blocks.Math.Gain gain1(k = 1 / 1032) annotation(
      Placement(transformation(extent = {{-80, 0}, {-60, 20}})));
  equation
    connect(serialReceive.pkgOut, unpackInt.pkgIn) annotation(
      Line(points = {{-159.2, 70}, {-150, 70}, {-150, 60.8}}));
    connect(unpackInt.y, integerToReal.u) annotation(
      Line(points = {{-139, 50}, {-122, 50}}, color = {255, 127, 0}, smooth = Smooth.None));
    connect(unpackInt.pkgOut[1], unpackInt1.pkgIn) annotation(
      Line(points = {{-150, 39.2}, {-150, 20.8}}, color = {0, 0, 0}, smooth = Smooth.None));
    connect(unpackInt1.y, integerToReal1.u) annotation(
      Line(points = {{-139, 10}, {-122, 10}}, color = {255, 127, 0}, smooth = Smooth.None));
    connect(integerToReal.y, gain.u) annotation(
      Line(points = {{-99, 50}, {-82, 50}}, color = {0, 0, 127}));
    connect(gain1.u, integerToReal1.y) annotation(
      Line(points = {{-82, 10}, {-99, 10}}, color = {0, 0, 127}));
    connect(gain.y, game.player1) annotation(
      Line(points = {{-59, 50}, {-28, 50}, {-28, 35}, {4, 35}}, color = {0, 0, 127}));
    connect(gain1.y, game.player2) annotation(
      Line(points = {{-59, 10}, {-28, 10}, {-28, 23}, {4, 23}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-180, -100}, {100, 100}}, preserveAspectRatio = false)),
      Icon(coordinateSystem(extent = {{-180, -100}, {100, 100}})));
  end Game_arduino;

  model Game
    parameter Real fieldHeight = 1;
    parameter Real fieldWidth = 2;
    parameter Real barLength = 0.5;
    parameter Real ballRad = 0.1;
    constant Real width = 0.05;
    parameter Real phi0 = 90 * Modelica.Constants.pi / 360;
    parameter Real vel0 = 0.5;
    Real p1, p2;
    Real ballX, ballY;
    Real vel(start = vel0);
    discrete Real phi(start = phi0);
    Real score1, score2;
    Real phi_deg = phi / Modelica.Constants.pi * 360;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape boundary_top(length = fieldWidth, width = width, height = width, color = {0, 0, 0}, r = {-fieldWidth / 2, fieldHeight / 2 + barLength / 2, 0}) annotation(
      Placement(transformation(extent = {{-20, 40}, {0, 60}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape boundary_bottom(length = fieldWidth, width = width, height = width, color = {0, 0, 0}, r = {-fieldWidth / 2, (-fieldHeight / 2) - barLength / 2, 0}) annotation(
      Placement(transformation(extent = {{-20, -40}, {0, -20}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape player1Bar(length = width, width = barLength, height = width, color = {0, 0, 255}, r = {-fieldWidth / 2, p1, 0}) annotation(
      Placement(transformation(extent = {{-60, 0}, {-40, 20}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape player2Bar(length = width, width = barLength, height = width, r = {fieldWidth / 2, p2, 0}, color = {0, 127, 0}) annotation(
      Placement(transformation(extent = {{20, 0}, {40, 20}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape shape(shapeType = "sphere", color = {255, 128, 0}, length = ballRad, width = ballRad, height = ballRad, r = {ballX, ballY, 0}) annotation(
      Placement(transformation(extent = {{-20, 0}, {0, 20}})));
    /*
          Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime
            synchronizeRealtime
            annotation (Placement(transformation(extent={{-140,-40},{-120,-20}})));
        */
    Modelica.Blocks.Interfaces.RealInput player1 "from 0 to 1" annotation(
      Placement(transformation(extent = {{-200, 50}, {-160, 90}})));
    Modelica.Blocks.Interfaces.RealInput player2 "from 0 to 1" annotation(
      Placement(transformation(extent = {{-200, -70}, {-160, -30}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape scoreBar1(color = {255, 0, 0}, height = width, length = width, r = {-fieldWidth / 1.5, score1/20, 0}, width = score1/10 +0.1) annotation(
      Placement(transformation(extent = {{36, -80}, {56, -60}})));
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape scoreBar2(color = {255, 0, 0}, height = width, length = width, r = {fieldWidth / 1.5, score2/20, 0}, width = score2/10 +0.1) annotation(
      Placement(transformation(extent = {{60, -80}, {80, -60}})));
  equation
    p1 = (player1 - 0.5) * fieldHeight;
    p2 = (player2 - 0.5) * fieldHeight;
    der(ballY) = sin(phi) * vel;
    der(ballX) = cos(phi) * vel;
//increase velocity
    vel = vel0 + time * 0.02;
//hit
    when ballY >= fieldHeight / 2 + barLength / 2 then
      phi = -pre(phi);
    elsewhen ballY <= (-fieldHeight / 2) - barLength / 2 then
      phi = -pre(phi);
    elsewhen abs(ballX) > fieldWidth / 2 then
      phi = (-pre(phi)) + Modelica.Constants.pi;
    end when;
    when abs(ballY - p1) > barLength / 2 and ballX <= (-fieldWidth / 2) then
      //Modelica.Utilities.Streams.print("Player2 hit" + String(time) + "\n");
      score2 = pre(score2) + 1;
      score1 = pre(score1);
      reinit(ballX, 0);
      reinit(ballY, 0);
    elsewhen abs(ballY - p2) > barLength / 2 and ballX >= fieldWidth / 2 then
      //Modelica.Utilities.Streams.print("Player1 hit" + String(time) + "\n");
      score1 = pre(score1) + 1;
      score2 = pre(score2);
      reinit(ballX, 0);
      reinit(ballY, 0);
    end when;
    annotation(
      Diagram(coordinateSystem(extent = {{-180, -100}, {100, 100}})),
      Icon(coordinateSystem(extent = {{-180, -100}, {100, 100}})));
  end Game;
  annotation(
    uses(Modelica(version = "3.2.1"), Modelica_DeviceDrivers(version = "1.4.4")));
end ModeliPong;
