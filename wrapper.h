#ifndef WRAPPER_H
#define WRAPPER_H

#include <XPLMPlugin.h>

PLUGIN_API float StartPerfFlightLoopCallback(
        float inElapsedSinceLastCall,
        float inElapsedTimeSinceLastFlightLoop,
        int inCounter,
        void *inRefcon);

PLUGIN_API float PrePerfFlightLoopCallback(
        float inElapsedSinceLastCall,
        float inElapsedTimeSinceLastFlightLoop,
        int inCounter,
        void *inRefcon);

PLUGIN_API float AfterPerfFlightLoopCallback(
        float inElapsedSinceLastCall,
        float inElapsedTimeSinceLastFlightLoop,
        int inCounter,
        void *inRefcon);

PLUGIN_API void XPluginMenuHandler(
        void * mRef,
        void * iRef);

#endif // WRAPPER_H
