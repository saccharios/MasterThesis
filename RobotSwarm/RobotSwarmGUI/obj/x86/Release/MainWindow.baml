   M S B A M L   `   `   `  ����   RobotSwarmGUI    RobotSwarmGUI.MainWindow   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -   $��myWindow��  � Title$  
RobotSwarm��$	��640��$	��480�� � Icon$ images/epuck.jpg������  �� ImageSource$ images/epuck.jpg��.���  � RowDefinitions �� $��2* ��� $��* � � ColumnDefinitions �� $��* ��� $��* �.+�o� -   $��menu1��$��Stretch=�$	��Top=�$��19��$��2��.@�m� $��_Menu��.@�m� -   $��_List Devices�� � ToolTip _� .���� .+��� $
��Bold"�List Devices�� *(Lists the robots that are already pairedm� -   $��_Proper Search�� _� .���� .+��� $
��Bold"�Proper Search�� CAScans the environment for active robots and adds them to the listm� $��_Add Paired Robot��.@�m� -   $��KheperaHeader��$��_Khepera��m� -   $��EpuckHeader��$��_Epuck��m� -   $��HelpMenu��$��_Help��.@�m� -   $��MenuDisableHelp��$��_Disable Help�� m� IsCheckable . � $��0��$��0�� �� $��4* ��� $��* � �� $	��80* ��� $
��229* �.+��� -	   $��listBox1��$��60,42,48,34q�$
��Auto=�$��2��$��2�����	#AED0EDF2���� *PairedDeviceCollection+ � 	Resources ��%C   (��       * SystemColors.HighlightBrushKey+)��Transparent������ .R���  �� Orientation$ 
Horizontal=�.+��� $��checkboxDevice��-
   $��10q����� *
DeviceName+	 d� 	IsChecked	 �� *	IsChecked+A� $��connectionInfo��$��10��$��10������ *ConnectionState+�� $��runInfo��$��Center=�$��Center=����� *	Algorithm+�� $��1��$��Center=�$��2���� $��1��$��0��$��Center=�.+��� $ 
Horizontal=�$��Center=�.+��� -   $��connectButton��$��Connect��$��23��$��75��$
��Left=�$	��Top=�$��10q�
 ^� 	IsEnabled
 �� *HelpEnabled+ _� .���� .+��� $
��Bold"�	Connect�� Connect a robot�� -   $��disconnectButton��$��
Disconnect��$��23��$��75��$
��Left=�$	��Top=�$��10q� _� .���� .+��� $
��Bold"�
Disconnect�� Disconnect a robot�� $ 
Horizontal=�$��Center=�.+��� -   $��startButton��$��Start��$��23��$��75��$��Center=�$��Center=�$��10q� _� .���� .+��� $
��Bold"�Start�� Start a robot�� -   $��
stopButton��$
��Stop��$��23��$��75��$��Center=�$��Center=�$��10q� _� .���� .+��� $
��Bold"�Stop�� Stop a robot�� $ 
Horizontal=�$��Center=�.+��� -   $��removeButton��$��Unpair��$��23��$��75��$��Center=�$��Center=�$��10q� _� .���� .+��� $
��Bold"�Remove�� -+Remove a robot from the list and unpair it.�� -   $��calibrateButton��$��	Calibrate��$��23��$��75��$��Center=�$��Center=�$��10q�{�.  _� .���� .+��� $
��Bold"�	Calibrate�� .,Calibrate the proximity sensors of the robot�� -   $��AlgComboBox��$��1��$
��Left=�$	��Top=�$	��200��$��	32,42,0,0q����� *AlgorithmCollection+���� .R��� ���� *	AlgName+�� -   $��InfoBox��$��1��$��1��$��Stretch=�$
��10,0q����	#ADD0EDF2���� *
InfoText+�� -   $��listBox2��$��1��$
��Auto=�$	��162��$
��Left=�$��	34,94,0,0q�$	��Top=�$	��180�����	#AED0EDF2���� *RemovedRoboBlobsCollection+ ��%C   (��       * SystemColors.HighlightBrushKey+)��Transparent���� -   $��label1��$��Removed Robot List��$��1��$��25��$
��Left=�$��	32,70,0,0q�$	��Top=�$	��172��$��12(�$
��Bold"����	#FFE5CDA4