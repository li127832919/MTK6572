##
## Copyright (C) 2012 The Android Open Source Project
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##      http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.
##

LOCAL_PATH:= $(call my-dir)

#############################################################
#   build the harfbuzz library
#

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm

LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES:= \
    src/Bidi.cc \
    src/CachedFace.cc \
    src/CmapCache.cc \
    src/Code.cc \
    src/direct_machine.cc \
    src/Face.cc \
    src/FeatureMap.cc \
    src/FileFace.cc \
    src/Font.cc \
    src/GlyphCache.cc \
    src/GlyphFace.cc \
    src/gr_char_info.cc \
    src/gr_face.cc \
    src/gr_features.cc \
    src/gr_font.cc \
    src/gr_logging.cc \
    src/gr_segment.cc \
    src/gr_slot.cc \
    src/json.cc \
    src/Justifier.cc \
    src/NameTable.cc \
    src/Pass.cc \
    src/SegCache.cc \
    src/SegCacheEntry.cc \
    src/SegCacheStore.cc \
    src/Segment.cc \
    src/Silf.cc \
    src/Slot.cc \
    src/Sparse.cc \
    src/TtfUtil.cc \
    src/UtfCodec.cc

LOCAL_CPP_EXTENSION := .cc

LOCAL_SHARED_LIBRARIES := \
        libstdc++ \
        libstlport \
        libcutils \
        libutils

LOCAL_C_INCLUDES += \
        bionic \
        external/stlport/stlport \
        $(LOCAL_PATH)/include \
        $(LOCAL_PATH)/src

LOCAL_CFLAGS += 

LOCAL_LDLIBS += -lpthread

LOCAL_MODULE:= libgraphite2

include $(BUILD_SHARED_LIBRARY)


