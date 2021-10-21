#include <cstdlib>
#include <QDebug>
#include <XPLMPlugin.h>
#include <XPLMDataAccess.h>
#include <XPLMProcessing.h>
#include "perfplugin.h"
#include "wrapper.h"
#include "debug.h"

static PerfPlugin *pPlugin = nullptr;

//Optional call backs
PLUGIN_API float StartPerfFlightLoopCallback(
        float inElapsedSinceLastCall,
        float inElapsedTimeSinceLastFlightLoop,
        int inCounter,
        void *inRefcon) {
    //CONSOLE("OldLoop");
    //qDebug() << "Wrapper old";
    if(pPlugin)
        return pPlugin->startFlightLoop(inElapsedSinceLastCall, inElapsedTimeSinceLastFlightLoop, inCounter, inRefcon);
    return 0.0f;
}

PLUGIN_API float PrePerfFlightLoopCallback(
        float inElapsedSinceLastCall,
        float inElapsedTimeSinceLastFlightLoop,
        int inCounter,
        void *inRefcon) {
    //CONSOLE("PreLoop");
    //qDebug() << "Wrapper pre";
    if(pPlugin)
        return pPlugin->preFlightLoop(inElapsedSinceLastCall, inElapsedTimeSinceLastFlightLoop, inCounter, inRefcon);
    return 0.0f;
}

PLUGIN_API float AfterPerfFlightLoopCallback(
        float inElapsedSinceLastCall,
        float inElapsedTimeSinceLastFlightLoop,
        int inCounter,
        void *inRefcon) {
    //CONSOLE("AfterLoop");
    //qDebug() << "Wrapper after";
    if(pPlugin)
        return pPlugin->afterFlightLoop(inElapsedSinceLastCall, inElapsedTimeSinceLastFlightLoop, inCounter, inRefcon);
    return 0.0f;
}

//Menu handler
PLUGIN_API void XPluginMenuHandler(
        void * mRef,
        void * iRef) {
//    XPLMDebugString("PerfPlugin MenuhHandler");
    CONSOLE("PerfPlugin MenuhHandler");
    if(pPlugin) {
        CONSOLE("call class menuhandler");
        return pPlugin->menuHandler(mRef, (char *) iRef);
    }
}

/*
PLUGIN_API void PerfMenuHandler(void *mRef, void *iRef) {
    if(pPlugin)
        return pPlugin->menuHandler(mRef, (char *) iRef);
    return;
}
*/

//Mandatory plugin start
PLUGIN_API int XPluginStart(
        char *outName,
        char *outSig,
        char *outDesc) {
    Q_ASSERT(!pPlugin);
    pPlugin = PerfPlugin::get();
    return pPlugin->pluginStart(outName, outSig, outDesc);
}

//Mandatory plugin stop
PLUGIN_API void XPluginStop() {
    DEBUG;
    //XPLMUnregisterFlightLoopCallback(PerfFlightLoopCallback, 0);
    pPlugin->pluginStop();
    delete pPlugin;
    pPlugin = nullptr;
    //qApp->exit();
}

//Mandatory plugin enable (initiate network etc)
PLUGIN_API int XPluginEnable() {
    DEBUG;
    if(pPlugin)
        return pPlugin->pluginEnable();
    return 0;
}

//Mandatory plugin disable (close connections)
PLUGIN_API void XPluginDisable() {
    DEBUG;
    if(pPlugin)
        pPlugin->pluginDisable();
}

//Mandatory message handler
PLUGIN_API void XPluginReceiveMessage(
        XPLMPluginID inFromWho,
        long inMessage,
        void *inParam) {
    if(pPlugin)
        return pPlugin->receivedMessage(inFromWho, inMessage, inParam);
}

