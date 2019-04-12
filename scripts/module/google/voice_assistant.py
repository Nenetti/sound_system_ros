# -*- coding: utf-8 -*-

import concurrent.futures
import json
import grpc

from google.assistant.embedded.v1alpha2 import embedded_assistant_pb2, embedded_assistant_pb2_grpc
from tenacity import retry, stop_after_attempt, retry_if_exception
from googlesamples.assistant.grpc import assistant_helpers, browser_helpers

END_OF_UTTERANCE = embedded_assistant_pb2.AssistResponse.END_OF_UTTERANCE
DIALOG_FOLLOW_ON = embedded_assistant_pb2.DialogStateOut.DIALOG_FOLLOW_ON
CLOSE_MICROPHONE = embedded_assistant_pb2.DialogStateOut.CLOSE_MICROPHONE
PLAYING = embedded_assistant_pb2.ScreenOutConfig.PLAYING


class VoiceAssistant(object):
    """Sample Assistant that supports conversations and device actions.

    Args:
      device_model_id: identifier of the device model.
      device_id: identifier of the registered device instance.
      conversation_stream(ConversationStream): audio stream
        for recording query and playing back assistant answer.
      channel: authorized gRPC channel for connection to the
        Google Assistant API.
      deadline_sec: gRPC deadline in seconds for Google Assistant API call.
      device_handler: callback for device actions.
    """

    def __init__(self, language_code, device_model_id, device_id,
                 conversation_stream, display,
                 channel, deadline_sec, device_handler, is_debug, is_answer):
        self.language_code = language_code
        self.device_model_id = device_model_id
        self.device_id = device_id
        self.conversation_stream = conversation_stream
        self.display = display
        self.is_debug = is_debug
        self.is_answer = is_answer

        # Opaque blob provided in AssistResponse that,
        # when provided in a follow-up AssistRequest,
        # gives the Assistant a context marker within the current state
        # of the multi-Assist()-RPC "conversation".
        # This value, along with MicrophoneMode, supports a more natural
        # "conversation" with the Assistant.
        self.conversation_state = None
        # Force reset of first conversation.
        self.is_new_conversation = True

        # Create Google Assistant API gRPC client.
        self.assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(channel)
        self.deadline = deadline_sec

        self.device_handler = device_handler

    ####################################################################################################
    def __enter__(self):
        return self

    ####################################################################################################
    def __exit__(self, etype, e, traceback):
        if e:
            return False
        self.conversation_stream.close()

    ####################################################################################################
    def stop(self):
        self.conversation_stream.stop_recording()

    ####################################################################################################
    def is_grpc_error_unavailable(e):
        is_grpc_error = isinstance(e, grpc.RpcError)
        if is_grpc_error and (e.code() == grpc.StatusCode.UNAVAILABLE):
            print('grpc unavailable error: %s', e)
            return True
        return False

    ####################################################################################################
    #
    #   音声認識
    #
    ####################################################################################################
    @retry(reraise=True, stop=stop_after_attempt(3),
           retry=retry_if_exception(is_grpc_error_unavailable))
    def start(self):
        """Send a voice request to the Assistant and playback the response.

        Returns: True if conversation should continue.
        """
        recognition_result = ""
        answer = ""
        device_actions_futures = []

        self.conversation_stream.start_recording()
        print('... Recording audio request ...')

        def iter_log_assist_requests():
            for c in self.gen_assist_requests():
                assistant_helpers.log_assist_request_without_audio(c)
                yield c
            # print('Reached end of AssistRequest iteration.')

        def get_result(response):
            # type: (google.assistant.embedded.v1alpha2.embedded_assistant_pb2.AssistResponse) -> Recognition_Result
            words = ""
            for word in response.speech_results:
                words += word.transcript
            return words

        # This generator yields AssistResponse proto messages
        # received from the gRPC Google Assistant API.
        for resp in self.assistant.Assist(iter_log_assist_requests(),
                                          self.deadline):
            assistant_helpers.log_assist_response_without_audio(resp)

            if resp.speech_results:
                recognition_result = get_result(resp)
                if self.is_debug:
                    print("[Debug] %s" % recognition_result)

            if resp.dialog_state_out.supplemental_display_text:
                answer = resp.dialog_state_out.supplemental_display_text
                if self.is_debug:
                    print("[Speech] [ %s ]\n" % recognition_result)
                    print("[Answer] [ %s ]\n" % answer)
                if not self.is_answer:
                    self.conversation_stream.stop_recording()
                    return recognition_result, answer

            # ユーザー発話終了
            if resp.event_type == END_OF_UTTERANCE:
                if not self.is_debug and not self.is_answer:
                    return recognition_result, answer

                # print('End of audio request detected.')
                # print('Stopping recording.')

                # if not self.is_answer:
                #    self.conversation_stream.stop_recording()
                #    return recognition_result

            # アシスタントからの返答再生
            if self.is_answer:
                if len(resp.audio_out.audio_data) > 0:
                    if not self.conversation_stream.playing:
                        self.conversation_stream.stop_recording()
                        self.conversation_stream.start_playback()
                        # print('Playing assistant response.')
                    # 音声再生部分
                    self.conversation_stream.write(resp.audio_out.audio_data)

            if resp.dialog_state_out.conversation_state:
                conversation_state = resp.dialog_state_out.conversation_state
                print('Updating conversation state.')
                self.conversation_state = conversation_state
            if resp.dialog_state_out.volume_percentage != 0:
                volume_percentage = resp.dialog_state_out.volume_percentage
                print('Setting volume to %s%%', volume_percentage)
                self.conversation_stream.volume_percentage = volume_percentage
            if resp.dialog_state_out.microphone_mode == DIALOG_FOLLOW_ON:
                continue_conversation = True
                print('Expecting follow-on query from user.')
            elif resp.dialog_state_out.microphone_mode == CLOSE_MICROPHONE:
                continue_conversation = False
            if resp.device_action.device_request_json:
                device_request = json.loads(
                    resp.device_action.device_request_json
                )
                fs = self.device_handler(device_request)
                if fs:
                    device_actions_futures.extend(fs)
            if self.display and resp.screen_out.data:
                system_browser = browser_helpers.system_browser
                system_browser.display(resp.screen_out.data)

        if len(device_actions_futures):
            #print('Waiting for device executions to complete.')
            concurrent.futures.wait(device_actions_futures)

        print('Finished playing assistant response.')
        if self.conversation_stream.playing:
            self.conversation_stream.stop_playback()
        return recognition_result, answer

    def gen_assist_requests(self):
        """Yields: AssistRequest messages to send to the API."""

        config = embedded_assistant_pb2.AssistConfig(
            audio_in_config=embedded_assistant_pb2.AudioInConfig(
                encoding='LINEAR16',
                sample_rate_hertz=self.conversation_stream.sample_rate,
            ),
            audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                encoding='LINEAR16',
                sample_rate_hertz=self.conversation_stream.sample_rate,
                volume_percentage=self.conversation_stream.volume_percentage,
            ),
            dialog_state_in=embedded_assistant_pb2.DialogStateIn(
                language_code=self.language_code,
                conversation_state=self.conversation_state,
                is_new_conversation=self.is_new_conversation,
            ),
            device_config=embedded_assistant_pb2.DeviceConfig(
                device_id=self.device_id,
                device_model_id=self.device_model_id,
            )
        )
        if self.display:
            config.screen_out_config.screen_mode = PLAYING
        # Continue current conversation with later requests.
        self.is_new_conversation = False
        # The first AssistRequest must contain the AssistConfig
        # and no audio data.
        yield embedded_assistant_pb2.AssistRequest(config=config)
        for data in self.conversation_stream:
            # Subsequent requests need audio data, but not config.
            yield embedded_assistant_pb2.AssistRequest(audio_in=data)
