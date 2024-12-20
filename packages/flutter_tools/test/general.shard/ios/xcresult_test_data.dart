// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// An example xcresult bundle json with invalid issues map.
const String kSampleResultJsonInvalidIssuesMap = r'''
{
  "_type" : {
    "_name" : "ActionsInvocationRecord"
  },
  "issues": []
}
''';

/// An example xcresult bundle json that contains warnings and errors that needs to be discarded per https://github.com/flutter/flutter/issues/95354.
const String kSampleResultJsonWithIssuesToBeDiscarded = r'''
{
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    },
    "errorSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "documentLocationInCreatingWorkspace" : {
            "_type" : {
              "_name" : "DocumentLocation"
            },
            "concreteTypeName" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "DVTTextDocumentLocation"
            },
            "url" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "file:\/\/\/Users\/m\/Projects\/test_create\/ios\/Runner\/AppDelegate.m#CharacterRangeLen=0&CharacterRangeLoc=263&EndingColumnNumber=56&EndingLineNumber=7&LocationEncoding=1&StartingColumnNumber=56&StartingLineNumber=7"
            }
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Semantic Issue"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Use of undeclared identifier 'asdas'"
          }
        },
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Uncategorized"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Command PhaseScriptExecution failed with a nonzero exit code"
          }
        }
      ]
    },
    "warningSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Warning"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99."
          }
        }
      ]
    }
  }
}
''';

/// An example xcresult bundle json that contains some warning and some errors.
const String kSampleResultJsonWithIssues = r'''
{
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    },
    "errorSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "documentLocationInCreatingWorkspace" : {
            "_type" : {
              "_name" : "DocumentLocation"
            },
            "concreteTypeName" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "DVTTextDocumentLocation"
            },
            "url" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "file:\/\/\/Users\/m\/Projects\/test_create\/ios\/Runner\/AppDelegate.m#CharacterRangeLen=0&CharacterRangeLoc=263&EndingColumnNumber=56&EndingLineNumber=7&LocationEncoding=1&StartingColumnNumber=56&StartingLineNumber=7"
            }
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Semantic Issue"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Use of undeclared identifier 'asdas'"
          }
        }
      ]
    },
    "warningSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Warning"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99."
          }
        }
      ]
    }
  }
}
''';

/// An example xcresult bundle json that contains some warning and some errors.
const String kSampleResultJsonWithNoProvisioningProfileIssue = r'''
{
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    },
    "errorSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "documentLocationInCreatingWorkspace" : {
            "_type" : {
              "_name" : "DocumentLocation"
            },
            "concreteTypeName" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "DVTTextDocumentLocation"
            },
            "url" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "file:\/\/\/Users\/m\/Projects\/test_create\/ios\/Runner\/AppDelegate.m#CharacterRangeLen=0&CharacterRangeLoc=263&EndingColumnNumber=56&EndingLineNumber=7&LocationEncoding=1&StartingColumnNumber=56&StartingLineNumber=7"
            }
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Error"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Runner requires a provisioning profile. Select a provisioning profile in the Signing & Capabilities editor"
          }
        }
      ]
    },
    "warningSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Warning"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99."
          }
        }
      ]
    }
  }
}
''';

/// An example xcresult bundle json that contains some warning and some errors.
const String kSampleResultJsonWithIssuesAndInvalidUrl = r'''
{
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    },
    "errorSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "documentLocationInCreatingWorkspace" : {
            "_type" : {
              "_name" : "DocumentLocation"
            },
            "concreteTypeName" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "DVTTextDocumentLocation"
            },
            "url" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "3:00"
            }
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Semantic Issue"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Use of undeclared identifier 'asdas'"
          }
        }
      ]
    },
    "warningSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Warning"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99."
          }
        }
      ]
    }
  }
}
''';

/// An example xcresult bundle json that contains no issues.
const String kSampleResultJsonNoIssues = r'''
{
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    }
  }
}
''';

/// An example xcresult bundle json with some provision profile issue.
const String kSampleResultJsonWithProvisionIssue = r'''
{
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    },
    "errorSummaries" : {
      "_type" : {
        "_name" : "Array"
      },
      "_values" : [
        {
          "_type" : {
            "_name" : "IssueSummary"
          },
          "issueType" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Semantic Issue"
          },
          "message" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Some Provisioning profile issue."
          }
        }
      ]
    }
  }
}
''';

/// An example xcresult bundle json that contains action issues.
const String kSampleResultJsonWithActionIssues = r'''
{
  "_type" : {
    "_name" : "ActionsInvocationRecord"
  },
  "actions" : {
    "_type" : {
      "_name" : "Array"
    },
    "_values" : [
      {
        "_type" : {
          "_name" : "ActionRecord"
        },
        "actionResult" : {
          "_type" : {
            "_name" : "ActionResult"
          },
          "coverage" : {
            "_type" : {
              "_name" : "CodeCoverageInfo"
            }
          },
          "issues" : {
            "_type" : {
              "_name" : "ResultIssueSummaries"
            },
            "testFailureSummaries" : {
              "_type" : {
                "_name" : "Array"
              },
              "_values" : [
                {
                  "_type" : {
                    "_name" : "TestFailureIssueSummary",
                    "_supertype" : {
                      "_name" : "IssueSummary"
                    }
                  },
                  "issueType" : {
                    "_type" : {
                      "_name" : "String"
                    },
                    "_value" : "Uncategorized"
                  },
                  "message" : {
                    "_type" : {
                      "_name" : "String"
                    },
                    "_value" : "Unable to find a destination matching the provided destination specifier:\n\t\t{ id:1234D567-890C-1DA2-34E5-F6789A0123C4 }\n\n\tIneligible destinations for the \"Runner\" scheme:\n\t\t{ platform:iOS, id:dvtdevice-DVTiPhonePlaceholder-iphoneos:placeholder, name:Any iOS Device, error:iOS 17.0 is not installed. To use with Xcode, first download and install the platform }"
                  }
                }
              ]
            }
          },
          "logRef" : {
            "_type" : {
              "_name" : "Reference"
            },
            "id" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "0~5X-qvql8_ppq0bj9taBMeZd4L2lXQagy1twsFRWwc06r42obpBZfP87uKnGO98mp5CUz1Ppr1knHiTMH9tOuwQ=="
            },
            "targetType" : {
              "_type" : {
                "_name" : "TypeDefinition"
              },
              "name" : {
                "_type" : {
                  "_name" : "String"
                },
                "_value" : "ActivityLogSection"
              }
            }
          },
          "metrics" : {
            "_type" : {
              "_name" : "ResultMetrics"
            }
          },
          "resultName" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "All Tests"
          },
          "status" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "failedToStart"
          },
          "testsRef" : {
            "_type" : {
              "_name" : "Reference"
            },
            "id" : {
              "_type" : {
                "_name" : "String"
              },
              "_value" : "0~Dmuz8-g6YRb8HPVbTUXJD21oy3r5jxIGi-njd2Lc43yR5JlJf7D78HtNn2BsrF5iw1uYMnsuJ9xFDV7ZAmwhGg=="
            },
            "targetType" : {
              "_type" : {
                "_name" : "TypeDefinition"
              },
              "name" : {
                "_type" : {
                  "_name" : "String"
                },
                "_value" : "ActionTestPlanRunSummaries"
              }
            }
          }
        },
        "buildResult" : {
          "_type" : {
            "_name" : "ActionResult"
          },
          "coverage" : {
            "_type" : {
              "_name" : "CodeCoverageInfo"
            }
          },
          "issues" : {
            "_type" : {
              "_name" : "ResultIssueSummaries"
            }
          },
          "metrics" : {
            "_type" : {
              "_name" : "ResultMetrics"
            }
          },
          "resultName" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "Build Succeeded"
          },
          "status" : {
            "_type" : {
              "_name" : "String"
            },
            "_value" : "succeeded"
          }
        },
        "endedTime" : {
          "_type" : {
            "_name" : "Date"
          },
          "_value" : "2023-07-10T12:52:22.592-0500"
        },
        "runDestination" : {
          "_type" : {
            "_name" : "ActionRunDestinationRecord"
          },
          "localComputerRecord" : {
            "_type" : {
              "_name" : "ActionDeviceRecord"
            },
            "platformRecord" : {
              "_type" : {
                "_name" : "ActionPlatformRecord"
              }
            }
          },
          "targetDeviceRecord" : {
            "_type" : {
              "_name" : "ActionDeviceRecord"
            },
            "platformRecord" : {
              "_type" : {
                "_name" : "ActionPlatformRecord"
              }
            }
          },
          "targetSDKRecord" : {
            "_type" : {
              "_name" : "ActionSDKRecord"
            }
          }
        },
        "schemeCommandName" : {
          "_type" : {
            "_name" : "String"
          },
          "_value" : "Test"
        },
        "schemeTaskName" : {
          "_type" : {
            "_name" : "String"
          },
          "_value" : "BuildAndAction"
        },
        "startedTime" : {
          "_type" : {
            "_name" : "Date"
          },
          "_value" : "2023-07-10T12:52:22.592-0500"
        },
        "title" : {
          "_type" : {
            "_name" : "String"
          },
          "_value" : "RunnerTests.xctest"
        }
      }
    ]
  },
  "issues" : {
    "_type" : {
      "_name" : "ResultIssueSummaries"
    }
  },
  "metadataRef" : {
    "_type" : {
      "_name" : "Reference"
    },
    "id" : {
      "_type" : {
        "_name" : "String"
      },
      "_value" : "0~pY0GqmiVE6Q3qlWdLJDp_PnrsUKsJ7KKM1zKGnvEZOWGdBeGNArjjU62kgF2UBFdQLdRmf5SGpImQfJB6e7vDQ=="
    },
    "targetType" : {
      "_type" : {
        "_name" : "TypeDefinition"
      },
      "name" : {
        "_type" : {
          "_name" : "String"
        },
        "_value" : "ActionsInvocationMetadata"
      }
    }
  },
  "metrics" : {
    "_type" : {
      "_name" : "ResultMetrics"
    }
  }
}
''';
