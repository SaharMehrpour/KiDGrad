   M S B A M L   `   `   `  ����  SkeletonBasics-WPF8    2Microsoft.Samples.Kinect.SkeletonBasics.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  KiDGraD��5   	   $	��950��6   $
��1080��6&    � DataContext 6a   ��  � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value  SquareCheckBox �� 
TargetType �� Orientation	 �Name
 ��Name ��Name �� TextAlignment �� Triggers P� Property P� Value �� 
TargetName��%1  &	        &	 #     &	 F     '�� i     &	 �     7  7  �� 5   
   $ 	#ff6e6e6e��62   �� 5   
   $ 	#ff52318f��64   �� 5	   
   $ 	#ff00BCF2��62   �� 5
   
   " ��6   .�5      ��  �@J�SnapsToDevicePixels . 64   5      �� 5   
   " ��6   .�5      ��  � �� 5      �� 5      " ��6&   .R�5      � .+�5      �� $ 
Horizontal=�6)   
������ .+�5   "   � $	 SquareCheckBoxChecked��6'   .+�5   &   �� $
 CheckedNormal��6,   $��Images\CheckedNormal.png��6C   $
��None=�6e   $��Center=�6t   �� 5   &   $
 CheckedHover��6,   $��Images\CheckedHover.png��6B   $
��None=�6c   $��Center=�6r   $y�	Collapsed=�6�   5   #   � 5   "   $	 SquareCheckBoxUnchecked��6'   $y�	Collapsed=�6H   .+�5   &   �� $
 UncheckedNormal��6,   $ ��Images\UncheckedNormal.png��6E   $
��None=�6i   $��Center=�6x   �� 5   &   $
 UncheckedHover��6,   $��Images\UncheckedHover.png��6D   $
��None=�6g   $��Center=�6v   $y�	Collapsed=�6�   5   #   �� 5   "   $ SquareCheckBoxText��6,   #��z��6H   $
 Left=�6i   $��Center=�6   8��  $��15(�6�   $��9,0,0,0q�6�   5      5       5      P� 5       �@d�	IsChecked$ falseY�6;   . �5    "   �� $ SquareCheckBoxChecked��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5!   "   $ SquareCheckBoxUnchecked��6O    � y�$ Visible=�6?   6O   5"      P� 5#       �@J�IsMouseOver . . �5$   "   �� $ CheckedNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5%   "   $ UncheckedNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5&   "   $ CheckedHover��6O    � y�$ Visible=�6?   6O   �� 5'   "   $ UncheckedHover��6O    � y�$ Visible=�6?   6O   �� 5(   "   8  $ SquareCheckBoxText��6h    � ��5)      5+      5-      5.      5/      .��51      � -   5        $��
layoutGrid��51      $��	10 0 10 0q�6    � ColumnDefinitions 52   
   �� 53      $
��628* �6    � RowDefinitions 55   
   �� 56      $
��100* �6   �� 57      $
��Auto �6   �� 58      $	��788 �6   .+�5:   
   ]� $��0,0,0,98q�6   6(   �� 5;   
   $��0��6   $4��./SkeletonBasics-WPF;component/Images/logo3.jpg��6   $
��None=�6U   $
��Left=�6d   $��
30,20,0,12q�6   $	��Top=�6�   �� 5<   
   -   5        $��currentCommand��5<   O   $��38��6   $
��Left=�6    $��
189,56,0,0q�6;   $��ADD NODE��6e   $	��Top=�6u   $��32(�6�   $	��285��6�   $ Center=�6�   $
��Bold"�6�   (� 5=   
   $��2��6    (� Stretch$
 None=�6   $
��Left=�6.   $��Bottom=�6I   $��	20,0,0,20q�6d   .��5>      �� -   5        $��Image1��5>      $	��960��6"   $	��720��6.    J� OpacityMask 6;   Z� 6U   5?      (� 5@   
   $��2��6   $
 None=�6   $
��Left=�6.   $��Bottom=�6I   $��	20,0,0,20q�6d   .��5A      �� -   5        $��Image��5A      $	��960��6!   $	��720��6-    J� Opacity$ 1Q�6Q    6:   Z� 6Q   5B      �� 5C   
   $��535,30,18,0q�6   $��Right=�6)   $	��Top=�6E   #��[  6]   $��Berlin Sans FB)�6�   $��26(�6�   $
��Bold"�6�   $��Normal$�6�   $��32��6�   $	��125��6�   	KiDGraD6�   �� 5D   
   -   5        $��
ResetImage��5D   W   $��2��6   $��75��6   $
��Left=�6)   $��	20,48,0,0q�6D   $
��Fill=�6i   $	��Top=�6x   $��75��6�   $4��./SkeletonBasics-WPF;component/Images/Reset.gif��6�   �� 5E   
   -   5        $��AddNodeImage��5E   X   $��2��6   $��75��6   $
��Left=�6)   $��
20,130,0,0q�6D   $
��Fill=�6l   $	��Top=�6{   $��75��6�   $<��6/SkeletonBasics-WPF;component/Images/AddNodeActive.gif��6�   �� 5F   
   -   5        $��RemoveNodeImage��5F   X   $��2��6   $��75��6   $
��Left=�6)   $��
20,215,0,0q�6D   $
��Fill=�6o   $	��Top=�6~   $��75��6�   $9��3/SkeletonBasics-WPF;component/Images/RemoveNode.gif��6�   �� 5G   
   -	   5        $��AddEdgeImage��5G   X   $��2��6   $��75��6   $
��Left=�6)   $��
20,299,0,0q�6D   $
��Fill=�6l   $	��Top=�6{   $��75��6�   $6��0/SkeletonBasics-WPF;component/Images/AddEdge.gif��6�   �� 5H   
   -
   5        $��RemoveEdgeImage��5H   X   $��2��6   $��75��6   $
��Left=�6)   $��
20,379,0,0q�6D   $
��Fill=�6o   $	��Top=�6~   $��75��6�   $9��3/SkeletonBasics-WPF;component/Images/RemoveEdge.gif��6�   5I      5L      