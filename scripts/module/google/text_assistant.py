# -*- coding: utf-8 -*-

"""Sample that implements a text client for the Google Assistant Service."""

import os
import logging
import json

import click
import google.auth.transport.grpc
import google.auth.transport.requests
import google.oauth2.credentials

from google.assistant.embedded.v1alpha2 import embedded_assistant_pb2, embedded_assistant_pb2_grpc
from googlesamples.assistant.grpc import assistant_helpers, browser_helpers

ASSISTANT_API_ENDPOINT = 'embeddedassistant.googleapis.com'
DEFAULT_GRPC_DEADLINE = 60 * 3 + 5
PLAYING = embedded_assistant_pb2.ScreenOutConfig.PLAYING


class TextAssistant(object):
    """Sample Assistant that supports text based conversations.
    Args:
      language_code: language for the conversation.
      device_model_id: identifier of the device model.
      device_id: identifier of the registered device instance.
      display: enable visual display of assistant response.
      channel: authorized gRPC channel for connection to the
        Google Assistant API.
      deadline_sec: gRPC deadline in seconds for Google Assistant API call.
    """

    def __init__(self, language_code, device_model_id, device_id,
                 display, channel, deadline_sec):
        self.language_code = language_code
        self.device_model_id = device_model_id
        self.device_id = device_id
        self.conversation_state = None
        self.is_new_conversation = True
        self.display = display
        self.assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(channel)
        self.deadline = deadline_sec

    def __enter__(self):
        return self

    def __exit__(self, etype, e, traceback):
        if e:
            return False

    def assist(self, text):
        """Send a text request to the Assistant and playback the response.
        """
        def iter_assist_requests(text):
            yield self.gen_assist_requests(text)

        text_response = None
        for resp in self.assistant.Assist(iter_assist_requests(text),
                                          self.deadline):
            assistant_helpers.log_assist_response_without_audio(resp)
            if resp.dialog_state_out.conversation_state:
                conversation_state = resp.dialog_state_out.conversation_state
                self.conversation_state = conversation_state
            if resp.dialog_state_out.supplemental_display_text:
                text_response = resp.dialog_state_out.supplemental_display_text
        return text_response

    def gen_assist_requests(self, text):
        """Yields: AssistRequest messages to send to the API."""

        config = embedded_assistant_pb2.AssistConfig(
            audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                encoding='LINEAR16',
                sample_rate_hertz=16000,
                volume_percentage=0,
            ),
            dialog_state_in=embedded_assistant_pb2.DialogStateIn(
                language_code=self.language_code,
                conversation_state=self.conversation_state,
                is_new_conversation=self.is_new_conversation,
            ),
            device_config=embedded_assistant_pb2.DeviceConfig(
                device_id=self.device_id,
                device_model_id=self.device_model_id,
            ),
            text_query=text,
        )
        # Continue current conversation with later requests.
        self.is_new_conversation = False
        if self.display:
            config.screen_out_config.screen_mode = PLAYING
        req = embedded_assistant_pb2.AssistRequest(config=config)
        assistant_helpers.log_assist_request_without_audio(req)
        yield req

    def start(api_endpoint, credentials,
             device_model_id, device_id, lang, display, verbose,
             grpc_deadline, *args, **kwargs):
        with SampleTextAssistant(lang, device_model_id, device_id, display,
                                 grpc_channel, grpc_deadline) as assistant:
            while True:
                query = click.prompt('')
                click.echo('<you> %s' % query)
                response_text, response_html = assistant.assist(text_query=query)
                if display and response_html:
                    system_browser = browser_helpers.system_browser
                    system_browser.display(response_html)
                if response_text:
                    click.echo('<@assistant> %s' % response_text)


if __name__ == '__main__':
    main()
