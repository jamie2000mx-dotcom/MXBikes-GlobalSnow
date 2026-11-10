/*
    Global Snow - MX Bikes telemetry foundation
    Based on the user-supplied MX Bikes output plugin example.

    This first milestone:
      - Receives live bike telemetry.
      - Detects front/rear wheel contact.
      - Estimates rear-wheel slip for later snow-spray effects.
      - Writes a CSV log in the plugin save folder.

    IMPORTANT:
      The MX Bikes output plugin API shown in mxb_example.c does not expose
      a 3D particle renderer or terrain-editing API. This DLL is a telemetry
      foundation, not yet a visible in-world snow renderer.
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

__declspec(dllexport) char *GetModID(void)
{
    return "mxbikes";
}

__declspec(dllexport) int GetModDataVersion(void)
{
    return 8;
}

__declspec(dllexport) int GetInterfaceVersion(void)
{
    return 9;
}

typedef struct
{
    int   m_iRPM;
    float m_fEngineTemperature;
    float m_fWaterTemperature;
    int   m_iGear;
    float m_fFuel;
    float m_fSpeedometer;
    float m_fPosX, m_fPosY, m_fPosZ;
    float m_fVelocityX, m_fVelocityY, m_fVelocityZ;
    float m_fAccelerationX, m_fAccelerationY, m_fAccelerationZ;
    float m_aafRot[3][3];
    float m_fYaw, m_fPitch, m_fRoll;
    float m_fYawVelocity, m_fPitchVelocity, m_fRollVelocity;
    float m_afSuspLength[2];
    float m_afSuspVelocity[2];
    int   m_iCrashed;
    float m_fSteer;
    float m_fThrottle;
    float m_fFrontBrake;
    float m_fRearBrake;
    float m_fClutch;
    float m_afWheelSpeed[2];
    int   m_aiWheelMaterial[2];
    float m_afBrakePressure[2];
    float m_fSteerTorque;
} SPluginsBikeData_t;

static FILE *g_log = NULL;
static char g_logPath[MAX_PATH] = { 0 };
static unsigned long g_sampleCount = 0;

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

__declspec(dllexport) int Startup(char *_szSavePath)
{
    g_sampleCount = 0;
    g_log = NULL;
    g_logPath[0] = '\0';

    if (_szSavePath != NULL && _szSavePath[0] != '\0')
    {
        size_t len = strlen(_szSavePath);
        if (len < MAX_PATH - 32)
        {
            strcpy_s(g_logPath, sizeof(g_logPath), _szSavePath);

            /* Ensure the folder separator is present. */
            if (g_logPath[len - 1] != '\\' && g_logPath[len - 1] != '/')
                strcat_s(g_logPath, sizeof(g_logPath), "\\");

            strcat_s(g_logPath, sizeof(g_logPath), "GlobalSnow_telemetry.csv");
            fopen_s(&g_log, g_logPath, "w");

            if (g_log != NULL)
            {
                fprintf(g_log,
                    "sample,bike_speed_mps,front_wheel_mps,rear_wheel_mps,"
                    "front_contact,rear_contact,throttle,rear_slip_mps,"
                    "spray_estimate,pos_x,pos_y,pos_z\n");
                fflush(g_log);
            }
        }
    }

    /* 3 = request 10 Hz, matching the supplied example's safe default. */
    return 3;
}

__declspec(dllexport) void Shutdown(void)
{
    if (g_log != NULL)
    {
        fclose(g_log);
        g_log = NULL;
    }
}

__declspec(dllexport) void EventInit(void *_pData, int _iDataSize) { (void)_pData; (void)_iDataSize; }
__declspec(dllexport) void EventDeinit(void) {}
__declspec(dllexport) void RunInit(void *_pData, int _iDataSize) { (void)_pData; (void)_iDataSize; }
__declspec(dllexport) void RunDeinit(void) {}
__declspec(dllexport) void RunStart(void) {}
__declspec(dllexport) void RunStop(void) {}
__declspec(dllexport) void RunLap(void *_pData, int _iDataSize) { (void)_pData; (void)_iDataSize; }
__declspec(dllexport) void RunSplit(void *_pData, int _iDataSize) { (void)_pData; (void)_iDataSize; }

__declspec(dllexport) void RunTelemetry(
    void *_pData,
    int _iDataSize,
    float _fTime,
    float _fPos)
{
    SPluginsBikeData_t *bike;
    float bikeSpeed;
    float rearWheelSpeed;
    float rearSlip;
    float sprayEstimate;
    int frontContact;
    int rearContact;

    (void)_fPos;

    if (_pData == NULL || _iDataSize < (int)sizeof(SPluginsBikeData_t))
        return;

    bike = (SPluginsBikeData_t *)_pData;

    bikeSpeed = fabsf(bike->m_fSpeedometer);
    rearWheelSpeed = fabsf(bike->m_afWheelSpeed[1]);

    frontContact = (bike->m_aiWheelMaterial[0] != 0) ? 1 : 0;
    rearContact  = (bike->m_aiWheelMaterial[1] != 0) ? 1 : 0;

    /*
       Approximate wheelspin indicator.
       This is only a telemetry estimate; it is not a physical snow model.
    */
    rearSlip = rearWheelSpeed - bikeSpeed;
    if (rearSlip < 0.0f)
        rearSlip = 0.0f;

    sprayEstimate = 0.0f;
    if (rearContact && bike->m_fThrottle > 0.05f)
    {
        sprayEstimate =
            clampf((rearSlip / 8.0f) * bike->m_fThrottle, 0.0f, 1.0f);
    }

    ++g_sampleCount;

    if (g_log != NULL)
    {
        fprintf(g_log,
            "%lu,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            g_sampleCount,
            bikeSpeed,
            fabsf(bike->m_afWheelSpeed[0]),
            rearWheelSpeed,
            frontContact,
            rearContact,
            clampf(bike->m_fThrottle, 0.0f, 1.0f),
            rearSlip,
            sprayEstimate,
            bike->m_fPosX,
            bike->m_fPosY,
            bike->m_fPosZ);

        /* Flush occasionally so the log remains useful if the game closes. */
        if ((g_sampleCount % 10) == 0)
            fflush(g_log);
    }

    (void)_fTime;
}

#ifdef __cplusplus
}
#endif
