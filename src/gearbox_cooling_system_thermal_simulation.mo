within ;
model Mutti220252_Assign2_Part_2

  Modelica.Thermal.FluidHeatFlow.Components.OpenTank SourceTank(
    medium=Modelica.Thermal.FluidHeatFlow.Media.Water_10degC(),
    T0=278.15,
    T0fixed=true,
    ATank=0.01,
    hTank=0.8,
    pAmbient(displayUnit="Pa") = 101325,
    useHeatPort=false,
    level(start=0.76, fixed=true))
    annotation (Placement(transformation(extent={{-120,-80},{-100,-60}})));
  Modelica.Thermal.FluidHeatFlow.Components.Pipe HeatExchanger(
    medium=Modelica.Thermal.FluidHeatFlow.Media.Water_10degC(),
    m=999.7*0.000502654824574367,
    T0=278.15,
    T0fixed=true,
    V_flow(start=0),
    frictionLoss=0,
    useHeatPort=true,
    h_g=0) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={0,-90})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow GearBoxHeatFlow(Q_flow=
        2414.4927)
    annotation (Placement(transformation(extent={{-120,30},{-100,50}})));
  Modelica.Thermal.HeatTransfer.Components.Convection Convection annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,40})));
  Modelica.Blocks.Sources.Constant AirConv(k=300) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-30,90})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor GearBoxWall(C=3000, T(
        start=389.8869, fixed=true))
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Modelica.Thermal.FluidHeatFlow.Components.OpenTank SinkTank(
    medium=Modelica.Thermal.FluidHeatFlow.Media.Water_10degC(),
    T0=278.15,
    T0fixed=true,
    ATank=0.01,
    hTank=0.8,
    pAmbient(displayUnit="Pa") = 101325,
    useHeatPort=true,
    level(start=0, fixed=true))
    annotation (Placement(transformation(extent={{40,-80},{60,-60}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor T_w annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-50,20})));
  Modelica.Thermal.FluidHeatFlow.Sources.VolumeFlow VolumetricPump(
    medium=Modelica.Thermal.FluidHeatFlow.Media.Water_10degC(),
    m=999.7*0.000502654824574367,
    T0=278.15,
    T0fixed=true,
    useVolumeFlowInput=true)
    annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
  Modelica.Blocks.Logical.Hysteresis Hysteresis(
    uLow=42 + 273.15,
    uHigh=58 + 273.15,
    pre_y_start=false)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-50,-10})));
  Modelica.Blocks.Sources.Constant ClosedFlow(k=0) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-80,-30})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-50,-50})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor Conduction(G=100)
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  Modelica.Blocks.Sources.Constant OpenFlow(k=2.5e-5) annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-20,-30})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow SinkTankCooler
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,-50})));
  Modelica.Blocks.Sources.Constant SinkTankT_ref(k=7.5 + 273.15) annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={110,-10})));
  Modelica.Blocks.Continuous.LimPID PI(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1200,
    Ti=9,
    yMax=0,
    yMin=-6000) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={70,-10})));
equation
  connect(AirConv.y, Convection.Gc)
    annotation (Line(points={{-30,79},{-30,50}}, color={0,0,127}));
  connect(GearBoxWall.port, Convection.solid)
    annotation (Line(points={{-50,50},{-50,40},{-40,40}}, color={191,0,0}));
  connect(Convection.fluid, HeatExchanger.heatPort)
    annotation (Line(points={{-20,40},{0,40},{0,-80}}, color={191,0,0}));
  connect(T_w.port, Convection.solid)
    annotation (Line(points={{-50,30},{-50,40},{-40,40}}, color={191,0,0}));
  connect(VolumetricPump.flowPort_a, SourceTank.flowPort) annotation (Line(
        points={{-60,-90},{-110,-90},{-110,-80}}, color={255,0,0}));
  connect(Hysteresis.y, switch1.u2)
    annotation (Line(points={{-50,-21},{-50,-38}},
                                                 color={255,0,255}));
  connect(Hysteresis.u, T_w.T)
    annotation (Line(points={{-50,2},{-50,9}},   color={0,0,127}));
  connect(switch1.y, VolumetricPump.volumeFlow)
    annotation (Line(points={{-50,-61},{-50,-80}}, color={0,0,127}));
  connect(Conduction.port_a, GearBoxHeatFlow.port)
    annotation (Line(points={{-80,40},{-100,40}},  color={191,0,0}));
  connect(Conduction.port_b, Convection.solid)
    annotation (Line(points={{-60,40},{-40,40}}, color={191,0,0}));
  connect(SinkTankCooler.port, SinkTank.heatPort)
    annotation (Line(points={{20,-60},{20,-80},{40,-80}}, color={191,0,0}));
  connect(SinkTank.TTank, PI.u_m)
    annotation (Line(points={{61,-76},{70,-76},{70,-22}}, color={0,0,127}));
  connect(SinkTankCooler.Q_flow, PI.y)
    annotation (Line(points={{20,-40},{20,-10},{59,-10}},
                                                      color={0,0,127}));
  connect(ClosedFlow.y, switch1.u3)
    annotation (Line(points={{-69,-30},{-58,-30},{-58,-38}}, color={0,0,127}));
  connect(OpenFlow.y, switch1.u1)
    annotation (Line(points={{-31,-30},{-42,-30},{-42,-38}}, color={0,0,127}));
  connect(PI.u_s, SinkTankT_ref.y)
    annotation (Line(points={{82,-10},{99,-10}},
                                             color={0,0,127}));
  connect(HeatExchanger.flowPort_b, SinkTank.flowPort)
    annotation (Line(points={{10,-90},{50,-90},{50,-80}}, color={255,0,0}));
  connect(VolumetricPump.flowPort_b, HeatExchanger.flowPort_a)
    annotation (Line(points={{-40,-90},{-10,-90}}, color={255,0,0}));
  annotation (uses(Modelica(version="4.0.0")),
    Diagram(coordinateSystem(extent={{-140,-120},{140,120}})),
    Icon(coordinateSystem(extent={{-140,-120},{140,120}})));
end Mutti220252_Assign2_Part_2;
