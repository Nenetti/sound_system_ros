#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Sample that implements a gRPC client for the Google Assistant API.

import os.path
import pathlib2 as pathlib
import time
import uuid

import click
import google.auth.transport.grpc
import google.auth.transport.requests
import google.oauth2.credentials
import os
import sys

import json

from voice_assistant import *
from text_assistant import *
from googlesamples.assistant.grpc import audio_helpers, device_helpers

ASSISTANT_API_ENDPOINT = 'embeddedassistant.googleapis.com'
IS_DISPLAY = False


####################################################################################################
#
#   本処理
#
####################################################################################################
def main(language, is_debug, is_answer, device_id=None, device_model_id=None):
    ####################################################################################################
    #
    #   初期定義
    #
    ####################################################################################################
    grpc_deadline = 60 * 3 + 5
    credentials = os.path.join(click.get_app_dir('google-oauthlib-tool'), 'credentials.json')
    device_config = os.path.join(click.get_app_dir('googlesamples-assistant'), 'device_config.json')
    audio_sample_rate = audio_helpers.DEFAULT_AUDIO_SAMPLE_RATE
    audio_sample_width = audio_helpers.DEFAULT_AUDIO_SAMPLE_WIDTH
    audio_iter_size = audio_helpers.DEFAULT_AUDIO_ITER_SIZE
    audio_block_size = audio_helpers.DEFAULT_AUDIO_DEVICE_BLOCK_SIZE
    audio_flush_size = audio_helpers.DEFAULT_AUDIO_DEVICE_FLUSH_SIZE
    ####################################################################################################
    #
    #   取り扱い説明書
    #
    ####################################################################################################
    """Samples for the Google Assistant API.

    Examples:
      Run the sample with microphone input and speaker output:

        $ python -m googlesamples.assistant

      Run the sample with file input and speaker output:

        $ python -m googlesamples.assistant -i <input file>

      Run the sample with file input and output:

        $ python -m googlesamples.assistant -i <input file> -o <output file>
    """
    ####################################################################################################
    #
    #   Load OAuth 2.0 CREDENTIALS.
    #   (認証)
    #
    ####################################################################################################
    try:
        with open(credentials, 'r') as f:
            credentials = google.oauth2.credentials.Credentials(token=None,
                                                                **json.load(f))
            http_request = google.auth.transport.requests.Request()
            credentials.refresh(http_request)
    except Exception as e:
        print('Error loading CREDENTIALS: %s' % e)
        print('Run google-oauthlib-tool to initialize ', 'new OAuth 2.0 CREDENTIALS.')
        sys.exit(-1)

    ####################################################################################################
    #
    #   Create an authorized gRPC channel.
    #   (grpc接続)
    #
    ####################################################################################################
    grpc_channel = google.auth.transport.grpc.secure_authorized_channel(
        credentials, http_request, ASSISTANT_API_ENDPOINT)
    # print("connection -> %s" % ASSISTANT_API_ENDPOINT)

    ####################################################################################################
    #
    #   Configure audio source and sink.
    #   (オーディオストリーム設定)
    #
    ####################################################################################################
    audio_device = None
    audio_sink = audio_source = (
            audio_device or audio_helpers.SoundDeviceStream(
        sample_rate=audio_sample_rate,
        sample_width=audio_sample_width,
        block_size=audio_block_size,
        flush_size=audio_flush_size
    )
    )

    ####################################################################################################
    #
    #   Create conversation stream with the given audio source and sink.
    #
    ####################################################################################################
    conversation_stream = audio_helpers.ConversationStream(
        source=audio_source,
        sink=audio_sink,
        iter_size=audio_iter_size,
        sample_width=audio_sample_width,
    )

    if device_id is None or device_model_id is None:
        try:
            with open(device_config) as f:
                device = json.load(f)
                device_id = device['id']
                device_model_id = device['model_id']
                print("Using device model %s and device id %s",
                      device_model_id,
                      device_id)

        except Exception as e:
            print(e)
            sys.exit(1)
    ####################################################################################################
    #
    #   コールバック設定
    #
    ####################################################################################################
    device_handler = device_helpers.DeviceRequestHandler(device_id)

    print(device_id, device_model_id)
    ####################################################################################################
    #
    #   アシスタント起動
    #
    ####################################################################################################
    '''
    text=None
    if text is not None:
        
    return TextAssistant(language, device_model_id, device_id, IS_DISPLAY,
                             grpc_channel, grpc_deadline)
'''
    return VoiceAssistant(language, device_model_id, device_id,
                          conversation_stream, IS_DISPLAY,
                          grpc_channel, grpc_deadline,
                          device_handler, is_debug, is_answer)
