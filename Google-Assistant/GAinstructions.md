# Register Custom On Device Actions

The following are instructions on how to set up Custom On Device Actions

## Getting Started

A Custom Device Action specifies the command that is sent to your device to trigger its special ability.
To define a Custom Device Action, you need four pieces of information:
1. A pattern to try to match against the user query
2. A Custom Device Action to associate with a matched query
3. Text that is spoken back to the user if the device supports the action
4. A command name that is sent back to the device, along with any parameters
You create the Custom Device Action by putting this information into an Actions on Google ([Action Package](https://developers.google.com/actions/reference/rest/Shared.Types/ActionPackage)). Action Packages define the format for Google Assistant Server responses. Unlike the Actions SDK, Custom Device Actions are fulfilled locally; you do not specify an endpoint to process requests and provide responses. Custom Device Actions are not conversational in nature.

### Create an Action Package

Create a file (e.g., actions.json) that defines a test command – open/close the gripper. You can use the example below:
```
{	
	{
            "name": "open gripper",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.OpenGripper",
                "trigger": {
                    "queryPatterns": [
                        "open gripper"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Ok, I am ready to pick up any object now!"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "OpenGripper"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "close gripper",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.CloseGripper",
                "trigger": {
                    "queryPatterns": [
                        "close gripper"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Okey-dokey!"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "CloseGripper"
                                }
                            }
                        ]
                    }
                }
            }
        },
         ]
            }
        }
    }
}
```
This example uses the following information to define the Custom Device Action:

1. A pattern to try to match against the user query (blink N times)
2. The Custom Device Action to associate with a matched query (com.example.actions.BlinkLight) for organizational purposes
3. Text that is spoken back to the user if the device supports the action (Blinking N times)
4. A command name (com.example.commands.BlinkLight) that is sent back to the device, along with any parameters (a number and possibly a description of the speed)

Note the following:

You can use Schema.org-defined types in the query pattern.
The types [...] array defines the list of custom types (for example, $Speed).
Custom types can be used in the query pattern. Any of the synonyms in that type can be spoken by the user to match the query pattern.
When a synonym does match, the type instance (speed) would return the normalized key (SLOWLY). There can be multiple entities in case, for example, there are different lights that support different speeds of blinking.
Parts of the request TTS pattern can be optional. For example, use ($Speed:speed)? in the query pattern to make this part optional.
$type.raw (for example, $speed.raw) in the response TTS is replaced by the word(s) the user actually spoke.
Descriptions for many of these fields are available in the Actions Package reference documentation.

## Deploy the Action Package

