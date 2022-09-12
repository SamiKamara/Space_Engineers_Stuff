/*
    v_0.1

    This is a Space Engineers script that resets one turrets rotation and sets other turrets to aim at same point
    as the master turret.

    Master turret is selected by naming it "Master"

    Based on Saitams() script which reseted all turrets to default position.
*/

void Main(string argument)
{
    TFMain();
}

private static string mastersName = "Master";
double _convergenceRange = 400d; //Distance to the point where the bullet streams touch

List<IMyTerminalBlock> TorretasGatlings = new List<IMyTerminalBlock>();
IMyShipController Ship;
List<IMyCockpit> Cockpits = new List<IMyCockpit>();
Vector3D _targetVec, _lastTargetVelocity = Vector3D.Zero;
IMyLargeTurretBase _designator;

public void TFMain()
{
    if(_designator != null)
    {
        _designator.ResetTargetingToDefault();
        _designator.Elevation = 0f;
        _designator.SyncElevation();
        _designator.Azimuth = 0f;
        _designator.SyncAzimuth();

        SlavedTurretTargeting();
    }
    else
    {
        Start();
    }
}

private void Start()
{
    List<IMyTerminalBlock> Terminales = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyShipController>(Terminales, b => b.CubeGrid == Me.CubeGrid);
    Ship = Terminales[0] as IMyShipController;

    GridTerminalSystem.GetBlocksOfType<IMyLargeTurretBase>(Terminales, b => b.CubeGrid == Me.CubeGrid);
    foreach (var y in Terminales) TorretasGatlings.Add(y as IMyLargeTurretBase);

    _designator = GridTerminalSystem.GetBlockWithName(mastersName) as IMyLargeTurretBase;
}

static Vector3D VectorAzimuthElevation(IMyLargeTurretBase turret)
{
    double el = turret.Elevation;
    double az = turret.Azimuth;
    Vector3D targetDirection;
    Vector3D.CreateFromAzimuthAndElevation(az, el, out targetDirection);
    return Vector3D.TransformNormal(targetDirection, turret.WorldMatrix);
}

void SlavedTurretTargeting()
{
    foreach (IMyLargeTurretBase thisTurret in TorretasGatlings)
    {
        Vector3D aimPosition = GetTargetPoint(thisTurret.GetPosition(), _designator);
        MatrixD turretMatrix = thisTurret.WorldMatrix;
        Vector3D turretDirection = VectorAzimuthElevation(thisTurret);
        Vector3D targetDirectionNorm = Vector3D.Normalize(aimPosition - turretMatrix.Translation);

        double azimuth = 0; double elevation = 0;
        GetRotationAngles(targetDirectionNorm, turretMatrix.Forward, turretMatrix.Left, turretMatrix.Up, out azimuth, out elevation);
        thisTurret.Azimuth = (float)azimuth;
        thisTurret.Elevation = (float)elevation;
        SyncTurretAngles(thisTurret);
    }
}

static void SyncTurretAngles(IMyLargeTurretBase turret)
{
    turret.SyncAzimuth(); //this syncs both angles
}

Vector3D GetTargetPoint(Vector3D shooterPosition, IMyLargeTurretBase designator)
{
    _targetVec = designator.GetPosition() + VectorAzimuthElevation(designator) * _convergenceRange;
    return _targetVec;
}

static void GetRotationAngles(Vector3D targetVector, Vector3D frontVec, Vector3D leftVec, Vector3D upVec, out double yaw, out double pitch)
{
    var matrix = MatrixD.Zero;
    matrix.Forward = frontVec; matrix.Left = leftVec; matrix.Up = upVec;

    var localTargetVector = Vector3D.TransformNormal(targetVector, MatrixD.Transpose(matrix));
    var flattenedTargetVector = new Vector3D(localTargetVector.X, 0, localTargetVector.Z);

    yaw = VectorMath.AngleBetween(Vector3D.Forward, flattenedTargetVector) * -Math.Sign(localTargetVector.X); //right is positive
    if (Math.Abs(yaw) < 1E-6 && localTargetVector.Z > 0) //check for straight back case
        yaw = Math.PI;

    if (Vector3D.IsZero(flattenedTargetVector)) //check for straight up case
        pitch = MathHelper.PiOver2 * Math.Sign(localTargetVector.Y);
    else
        pitch = VectorMath.AngleBetween(localTargetVector, flattenedTargetVector) * Math.Sign(localTargetVector.Y); //up is positive
}

public static class VectorMath
{
    public static Vector3D SafeNormalize(Vector3D a)
    {
        if (Vector3D.IsZero(a))
            return Vector3D.Zero;

        if (Vector3D.IsUnit(ref a))
            return a;

        return Vector3D.Normalize(a);
    }

    public static Vector3D Reflection(Vector3D a, Vector3D b, double rejectionFactor = 1) //reflect a over b
    {
        Vector3D project_a = Projection(a, b);
        Vector3D reject_a = a - project_a;
        return project_a - reject_a * rejectionFactor;
    }

    public static Vector3D Rejection(Vector3D a, Vector3D b) //reject a on b
    {
        if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
            return Vector3D.Zero;

        return a - a.Dot(b) / b.LengthSquared() * b;
    }

    public static Vector3D Projection(Vector3D a, Vector3D b)
    {
        if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
            return Vector3D.Zero;

        return a.Dot(b) / b.LengthSquared() * b;
    }

    public static double ScalarProjection(Vector3D a, Vector3D b)
    {
        if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
            return 0;

        if (Vector3D.IsUnit(ref b))
            return a.Dot(b);

        return a.Dot(b) / b.Length();
    }

    public static double AngleBetween(Vector3D a, Vector3D b) //returns radians
    {
        if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
            return 0;
        else
            return Math.Acos(MathHelper.Clamp(a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared()), -1, 1));
    }

    public static double CosBetween(Vector3D a, Vector3D b, bool useSmallestAngle = false) //returns radians
    {
        if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
            return 0;
        else
            return MathHelper.Clamp(a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared()), -1, 1);
    }

    public static bool IsDotProductWithinTolerance(Vector3D a, Vector3D b, double tolerance)
    {
        double dot = Vector3D.Dot(a, b);
        double num = a.LengthSquared() * b.LengthSquared() * tolerance * tolerance;
        return dot * dot > num;
    }
}

public static class StringExtensions
{
    public static bool Contains(string source, string toCheck, StringComparison comp = StringComparison.OrdinalIgnoreCase)
    {
        return source?.IndexOf(toCheck, comp) >= 0;
    }
}

