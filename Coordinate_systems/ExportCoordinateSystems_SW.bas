'==============================================================================
' ExportCoordinateSystems_SW.bas
'
' Exports the position + rotation of every ASSEMBLY-LEVEL coordinate system in
' the active SolidWorks assembly to a YAML file.
'
' Per coordinate system the macro writes:
'   - translation tx,ty,tz          (METERS)
'   - quaternion  qx,qy,qz,qw       (ROS / tf2 order, x,y,z,w)
'   - roll,pitch,yaw                (ZYX intrinsic, RADIANS)
'   - full 3x3 rotation matrix      (row-major r00..r22)
'
' BASE FRAME:
'   Set BASE_CS_NAME to the name of an existing coordinate-system feature
'   (e.g. "Vehicle_Base") to report every other system RELATIVE TO that frame.
'   Leave it "" to report poses relative to the assembly global origin.
'
' UNITS:
'   The SolidWorks API always works in METERS and RADIANS regardless of the
'   document's display units, so the output is SI (ready for ROS2 / Kalibr).
'
' VALIDATE BEFORE TRUSTING (do this once for your SW version / research log):
'   Create a test coordinate system with a KNOWN offset and a KNOWN single-axis
'   rotation (e.g. +90 deg about Z, translated +0.100 m in X). Run the macro and
'   confirm the YAML matches expectation. If the rotation comes out transposed,
'   swap the assignment in ExtractRT (marked below). This pins the convention.
'
' LOCALE:
'   All numbers are written with a period decimal separator (via Str) regardless
'   of the machine's regional settings, so the YAML is valid on German locales
'   too (no "0,003"-style commas).
'
' HOW TO RUN:
'   Tools > Macro > Edit... (or New), paste this into a module, set the
'   constants below, then Tools > Macro > Run and pick main. With ASSEMBLY_PATH
'   set, the macro opens that assembly itself; leave it "" to run against the
'   document already open and active.
'   Requires a reference to the SOLIDWORKS type library (default in the macro IDE).
'==============================================================================

Const BASE_CS_NAME As String = "base_link"                 ' "" = assembly origin
Const ASSEMBLY_PATH As String = ""  ' "" = use the active document
Const OUTPUT_PATH  As String = "C:\Users\carlo\OneDrive - TUM\Engineering Project CAD\Git_clone\Mobile-Mapping-Backpack\Coordinate_systems\coordinate_systems.yaml"

Dim swApp As SldWorks.SldWorks

Sub main()
    Set swApp = Application.SldWorks

    Dim swModel As SldWorks.ModelDoc2
    If Len(ASSEMBLY_PATH) > 0 Then
        Dim nErr As Long, nWarn As Long
        Set swModel = swApp.OpenDoc6(ASSEMBLY_PATH, _
                                     swDocumentTypes_e.swDocASSEMBLY, _
                                     swOpenDocOptions_e.swOpenDocOptions_Silent, _
                                     "", nErr, nWarn)
        If swModel Is Nothing Then
            MsgBox "Could not open assembly:" & vbCrLf & ASSEMBLY_PATH & _
                   vbCrLf & "Error code: " & nErr
            Exit Sub
        End If
    Else
        Set swModel = swApp.ActiveDoc
    End If

    If swModel Is Nothing Then
        MsgBox "Open an assembly first (or set ASSEMBLY_PATH)."
        Exit Sub
    End If
    If swModel.GetType <> swDocumentTypes_e.swDocASSEMBLY Then
        MsgBox "Active document is not an assembly."
        Exit Sub
    End If

    '---- collect all assembly-level coordinate systems --------------------
    Dim names()  As String
    Dim mats()   As Variant     ' each element = 16-double ArrayData
    Dim count    As Integer
    ReDim names(255)
    ReDim mats(255)
    count = 0

    Dim swFeat As SldWorks.Feature
    Set swFeat = swModel.FirstFeature
    Do While Not swFeat Is Nothing
        If swFeat.GetTypeName2() = "CoordSys" Then
            Dim swCS As SldWorks.CoordinateSystemFeatureData
            Set swCS = swFeat.GetDefinition
            Dim swXform As SldWorks.MathTransform
            Set swXform = swCS.Transform
            names(count) = swFeat.Name
            mats(count) = swXform.ArrayData
            count = count + 1
        End If
        Set swFeat = swFeat.GetNextFeature
    Loop

    If count = 0 Then
        MsgBox "No assembly-level coordinate systems found." & vbCrLf & _
               "If your systems live inside component parts, see the note at " & _
               "the bottom of the macro file."
        Exit Sub
    End If

    '---- resolve base frame if requested ----------------------------------
    Dim Rb(2, 2) As Double, tb(2) As Double
    Dim useBase As Boolean
    useBase = (Len(BASE_CS_NAME) > 0)
    If useBase Then
        Dim found As Boolean
        found = False
        Dim bi As Integer
        For bi = 0 To count - 1
            If names(bi) = BASE_CS_NAME Then
                ExtractRT mats(bi), Rb, tb
                found = True
                Exit For
            End If
        Next bi
        If Not found Then
            MsgBox "Base coordinate system '" & BASE_CS_NAME & "' not found."
            Exit Sub
        End If
    End If

    '---- write YAML --------------------------------------------------------
    Dim refFrame As String
    If useBase Then refFrame = BASE_CS_NAME Else refFrame = "ASSEMBLY_ORIGIN"

    Dim f As Integer
    f = FreeFile
    Open OUTPUT_PATH For Output As #f

    ' --- file header / metadata ---
    Print #f, "# Sensor coordinate systems exported from SolidWorks assembly"
    Print #f, "# Each pose is the sensor frame expressed in the reference frame below."
    Print #f, "# Quaternion order: x, y, z, w (ROS / tf2)"
    Print #f, "# RPY: ZYX intrinsic (roll about X, pitch about Y, yaw about Z)"
    Print #f, "reference_frame: " & refFrame
    Print #f, "units:"
    Print #f, "  translation: meters"
    Print #f, "  rotation: radians"
    Print #f, "frames:"

    Dim i As Integer
    For i = 0 To count - 1
        If useBase And names(i) = BASE_CS_NAME Then GoTo NextI   ' skip base vs itself

        Dim R(2, 2) As Double, t(2) As Double
        ExtractRT mats(i), R, t

        Dim Ro(2, 2) As Double, to_(2) As Double
        If useBase Then
            RelativePose Rb, tb, R, t, Ro, to_
        Else
            CopyR R, Ro
            to_(0) = t(0): to_(1) = t(1): to_(2) = t(2)
        End If

        Dim q(3) As Double          ' x, y, z, w
        RotToQuat Ro, q
        Dim rpy(2) As Double        ' roll, pitch, yaw (ZYX)
        RotToRPY Ro, rpy

        Print #f, "  """ & names(i) & """:"
        Print #f, "    parent: " & refFrame
        Print #f, "    translation: [" & NumStr(to_(0)) & ", " & NumStr(to_(1)) & ", " & NumStr(to_(2)) & "]"
        Print #f, "    quaternion: {x: " & NumStr(q(0)) & ", y: " & NumStr(q(1)) & ", z: " & NumStr(q(2)) & ", w: " & NumStr(q(3)) & "}"
        Print #f, "    rpy: [" & NumStr(rpy(0)) & ", " & NumStr(rpy(1)) & ", " & NumStr(rpy(2)) & "]"
        Print #f, "    rotation_matrix:"
        Print #f, "      - [" & NumStr(Ro(0, 0)) & ", " & NumStr(Ro(0, 1)) & ", " & NumStr(Ro(0, 2)) & "]"
        Print #f, "      - [" & NumStr(Ro(1, 0)) & ", " & NumStr(Ro(1, 1)) & ", " & NumStr(Ro(1, 2)) & "]"
        Print #f, "      - [" & NumStr(Ro(2, 0)) & ", " & NumStr(Ro(2, 1)) & ", " & NumStr(Ro(2, 2)) & "]"
NextI:
    Next i

    Close #f
    MsgBox count & " coordinate system(s) processed." & vbCrLf & _
           "Saved to: " & OUTPUT_PATH
End Sub

'==============================================================================
' Helpers
'==============================================================================

' Locale-independent number -> string. Str() always uses a period as the
' decimal separator (unlike CStr/Format, which follow regional settings), so
' this is safe on German locales. A ".0" is appended when needed so every
' value is unambiguously a YAML float (never parsed as an integer).
Function NumStr(x As Double) As String
    Dim s As String
    s = Trim(Str(x))
    If InStr(s, ".") = 0 And InStr(s, "E") = 0 And InStr(s, "e") = 0 Then
        s = s & ".0"
    End If
    NumStr = s
End Function

' ArrayData layout (SolidWorks):  rotation row-major in 0..8, translation 9..11.
'        0 1 2          tx = 9
'        3 4 5          ty = 10
'        6 7 8          tz = 11    (index 12 = scale; 13..15 unused)
'
' If your validation test shows the rotation transposed, swap each R(a,b) below
' for R(b,a) (i.e. read column-major instead) and re-run the test.
Sub ExtractRT(arr As Variant, R() As Double, t() As Double)
    R(0, 0) = arr(0): R(0, 1) = arr(1): R(0, 2) = arr(2)
    R(1, 0) = arr(3): R(1, 1) = arr(4): R(1, 2) = arr(5)
    R(2, 0) = arr(6): R(2, 1) = arr(7): R(2, 2) = arr(8)
    t(0) = arr(9): t(1) = arr(10): t(2) = arr(11)
End Sub

Sub CopyR(R() As Double, Ro() As Double)
    Dim a As Integer, b As Integer
    For a = 0 To 2
        For b = 0 To 2
            Ro(a, b) = R(a, b)
        Next b
    Next a
End Sub

' Pose of target frame expressed in the base frame:
'   R_rel = Rb^T * Rt
'   t_rel = Rb^T * (tt - tb)
Sub RelativePose(Rb() As Double, tb() As Double, Rt() As Double, tt() As Double, _
                 Ro() As Double, to_() As Double)
    Dim a As Integer, b As Integer, k As Integer, s As Double
    For a = 0 To 2
        For b = 0 To 2
            s = 0
            For k = 0 To 2
                s = s + Rb(k, a) * Rt(k, b)    ' Rb^T(a,k) = Rb(k,a)
            Next k
            Ro(a, b) = s
        Next b
    Next a

    Dim d(2) As Double
    d(0) = tt(0) - tb(0): d(1) = tt(1) - tb(1): d(2) = tt(2) - tb(2)
    For a = 0 To 2
        s = 0
        For k = 0 To 2
            s = s + Rb(k, a) * d(k)
        Next k
        to_(a) = s
    Next a
End Sub

' Rotation matrix -> quaternion (x,y,z,w). Shepperd's method (numerically stable).
Sub RotToQuat(R() As Double, q() As Double)
    Dim tr As Double, S As Double
    Dim qw As Double, qx As Double, qy As Double, qz As Double
    tr = R(0, 0) + R(1, 1) + R(2, 2)
    If tr > 0 Then
        S = Sqr(tr + 1#) * 2#
        qw = 0.25 * S
        qx = (R(2, 1) - R(1, 2)) / S
        qy = (R(0, 2) - R(2, 0)) / S
        qz = (R(1, 0) - R(0, 1)) / S
    ElseIf (R(0, 0) > R(1, 1)) And (R(0, 0) > R(2, 2)) Then
        S = Sqr(1# + R(0, 0) - R(1, 1) - R(2, 2)) * 2#
        qw = (R(2, 1) - R(1, 2)) / S
        qx = 0.25 * S
        qy = (R(0, 1) + R(1, 0)) / S
        qz = (R(0, 2) + R(2, 0)) / S
    ElseIf R(1, 1) > R(2, 2) Then
        S = Sqr(1# + R(1, 1) - R(0, 0) - R(2, 2)) * 2#
        qw = (R(0, 2) - R(2, 0)) / S
        qx = (R(0, 1) + R(1, 0)) / S
        qy = 0.25 * S
        qz = (R(1, 2) + R(2, 1)) / S
    Else
        S = Sqr(1# + R(2, 2) - R(0, 0) - R(1, 1)) * 2#
        qw = (R(1, 0) - R(0, 1)) / S
        qx = (R(0, 2) + R(2, 0)) / S
        qy = (R(1, 2) + R(2, 1)) / S
        qz = 0.25 * S
    End If
    q(0) = qx: q(1) = qy: q(2) = qz: q(3) = qw
End Sub

' Rotation matrix -> roll,pitch,yaw (ZYX intrinsic: yaw=Z, pitch=Y, roll=X).
Sub RotToRPY(R() As Double, rpy() As Double)
    Dim sy As Double
    sy = Sqr(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))
    If sy < 0.000001 Then               ' gimbal-lock branch
        rpy(0) = Atn2(-R(1, 2), R(1, 1))
        rpy(1) = Atn2(-R(2, 0), sy)
        rpy(2) = 0#
    Else
        rpy(0) = Atn2(R(2, 1), R(2, 2))
        rpy(1) = Atn2(-R(2, 0), sy)
        rpy(2) = Atn2(R(1, 0), R(0, 0))
    End If
End Sub

' VBA has no Atan2; provide one.
Function Atn2(y As Double, x As Double) As Double
    Const PI As Double = 3.14159265358979
    If x > 0 Then
        Atn2 = Atn(y / x)
    ElseIf x < 0 Then
        If y >= 0 Then
            Atn2 = Atn(y / x) + PI
        Else
            Atn2 = Atn(y / x) - PI
        End If
    Else
        If y > 0 Then
            Atn2 = PI / 2
        ElseIf y < 0 Then
            Atn2 = -PI / 2
        Else
            Atn2 = 0
        End If
    End If
End Function

'==============================================================================
' NOTE - coordinate systems defined INSIDE component parts
'
' This macro reads systems created at the assembly level (the usual place to
' define sensor-mount frames). If a coordinate system lives inside a component
' part, its CoordinateSystemFeatureData.Transform is relative to THAT part's
' origin. To bring it into assembly space, multiply by the component pose:
'
'     Set swComp   = <Component2 of the part>
'     Set swCompXf = swComp.Transform2          ' part-local -> assembly
'     ' assembly pose of the CS = swCompXf composed with the CS transform.
'
' Validate the composition order on a known test case before relying on it.
'==============================================================================
