within ;
model Mutti220252_Assign2_Part_1
  Modelica.Electrical.Analog.Sources.SignalVoltage V_in annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-40,40})));
  Modelica.Electrical.Analog.Basic.Resistor R_c(R=0.1, T_ref=298.15)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-26,60})));
  Modelica.Electrical.Analog.Basic.Inductor L_c(i(start=0, fixed=true), L=0.01)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={6,60})));
  Modelica.Electrical.Analog.Basic.RotationalEMF V_e(k=0.3)
    annotation (Placement(transformation(extent={{10,30},{30,50}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-8,10})));
  Modelica.Mechanics.Rotational.Components.Inertia J_m(J=0.001)
    annotation (Placement(transformation(extent={{40,30},{60,50}})));
  Modelica.Mechanics.Rotational.Components.LossyGear GearBox(
    ratio=2,
    lossTable=[0,0.99,0.99,0,0; 50,0.98,0.98,0.5,0.5; 100,0.97,0.97,1,1; 210,
        0.96,0.96,1.5,1.5],
    useHeatPort=true,
    locked(fixed=true))
    annotation (Placement(transformation(extent={{70,30},{90,50}})));
  Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque T_load(
    tau_nominal=-100,
    TorqueDirection=true,
    w_nominal=210,
    phi(start=0))
    annotation (Placement(transformation(extent={{70,60},{90,80}})));
  Modelica.Mechanics.Rotational.Components.Inertia J_p(
    J=2710*0.01^2*0.8*(0.8^2 + 0.01^2)/12,
    phi(fixed=true, start=0),
             w(start=0, fixed=true))
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Sources.Step RefVelocity(height=210, startTime=5) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-138,40})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor GearBoxWall(C=3000, T(
        start=298.15, fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={70,-20})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor Conduction(G=100)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={70,10})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-110,0})));
  Modelica.Blocks.Math.Feedback feedback annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-110,40})));
  Modelica.Blocks.Continuous.PI PI(
    k=0.03,
    T=0.025,
    initType=Modelica.Blocks.Types.Init.InitialState,
    x_start=0)
    annotation (Placement(transformation(extent={{-90,30},{-70,50}})));
equation
  connect(L_c.p, R_c.n)
    annotation (Line(points={{-4,60},{-16,60}}, color={0,0,255}));
  connect(L_c.n, V_e.p)
    annotation (Line(points={{16,60},{20,60},{20,50}}, color={0,0,255}));
  connect(J_m.flange_b, GearBox.flange_a)
    annotation (Line(points={{60,40},{70,40}}, color={0,0,0}));
  connect(GearBox.flange_b, J_p.flange_a)
    annotation (Line(points={{90,40},{100,40}}, color={0,0,0}));
  connect(T_load.flange, J_p.flange_b) annotation (Line(points={{90,70},{130,70},
          {130,40},{120,40}}, color={0,0,0}));
  connect(GearBox.heatPort, Conduction.port_b)
    annotation (Line(points={{70,30},{70,20}}, color={191,0,0}));
  connect(GearBoxWall.port, Conduction.port_a)
    annotation (Line(points={{70,-10},{70,0}}, color={191,0,0}));
  connect(RefVelocity.y, feedback.u1)
    annotation (Line(points={{-127,40},{-118,40}}, color={0,0,127}));
  connect(feedback.y, PI.u)
    annotation (Line(points={{-101,40},{-92,40}},color={0,0,127}));
  connect(V_in.v, PI.y)
    annotation (Line(points={{-52,40},{-69,40}}, color={0,0,127}));
  connect(V_e.n, ground.p)
    annotation (Line(points={{20,30},{20,20},{-8,20}}, color={0,0,255}));
  connect(R_c.p, V_in.p)
    annotation (Line(points={{-36,60},{-40,60},{-40,50}}, color={0,0,255}));
  connect(speedSensor.w, feedback.u2)
    annotation (Line(points={{-110,11},{-110,32}}, color={0,0,127}));
  connect(speedSensor.flange, J_p.flange_b) annotation (Line(points={{-110,-10},
          {-110,-40},{130,-40},{130,40},{120,40}}, color={0,0,0}));
  connect(V_e.flange, J_m.flange_a)
    annotation (Line(points={{30,40},{40,40}}, color={0,0,0}));
  connect(V_in.n, ground.p)
    annotation (Line(points={{-40,30},{-40,20},{-8,20}}, color={0,0,255}));
  annotation (uses(Modelica(version="4.0.0")),
    Diagram(coordinateSystem(extent={{-160,-60},{140,100}})),
    Icon(coordinateSystem(extent={{-160,-60},{140,100}})));
end Mutti220252_Assign2_Part_1;
