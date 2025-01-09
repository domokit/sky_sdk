// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:intl/intl.dart' as intl;

import 'gallery_localizations_en.dart';

/// Callers can lookup localized strings with an instance of GalleryLocalizations
/// returned by `GalleryLocalizations.of(context)`.
///
/// Applications need to include `GalleryLocalizations.delegate()` in their app's
/// `localizationDelegates` list, and the locales they support in the app's
/// `supportedLocales` list. For example:
///
/// ```dart
/// import 'gen_l10n/gallery_localizations.dart';
///
/// return MaterialApp(
///   localizationsDelegates: GalleryLocalizations.localizationsDelegates,
///   supportedLocales: GalleryLocalizations.supportedLocales,
///   home: MyApplicationHome(),
/// );
/// ```
///
/// ## Update pubspec.yaml
///
/// Please make sure to update your pubspec.yaml to include the following
/// packages:
///
/// ```yaml
/// dependencies:
///   # Internationalization support.
///   flutter_localizations:
///     sdk: flutter
///   intl: any # Use the pinned version from flutter_localizations
///
///   # Rest of dependencies
/// ```
///
/// ## iOS Applications
///
/// iOS applications define key application metadata, including supported
/// locales, in an Info.plist file that is built into the application bundle.
/// To configure the locales supported by your app, you’ll need to edit this
/// file.
///
/// First, open your project’s ios/Runner.xcworkspace Xcode workspace file.
/// Then, in the Project Navigator, open the Info.plist file under the Runner
/// project’s Runner folder.
///
/// Next, select the Information Property List item, select Add Item from the
/// Editor menu, then select Localizations from the pop-up menu.
///
/// Select and expand the newly-created Localizations item then, for each
/// locale your application supports, add a new item and select the locale
/// you wish to add from the pop-up menu in the Value field. This list should
/// be consistent with the languages listed in the GalleryLocalizations.supportedLocales
/// property.
abstract class GalleryLocalizations {
  GalleryLocalizations(String locale) : localeName = intl.Intl.canonicalizedLocale(locale);

  final String localeName;

  static GalleryLocalizations? of(BuildContext context) {
    return Localizations.of<GalleryLocalizations>(context, GalleryLocalizations);
  }

  static const LocalizationsDelegate<GalleryLocalizations> delegate =
      _GalleryLocalizationsDelegate();

  /// A list of this localizations delegate along with the default localizations
  /// delegates.
  ///
  /// Returns a list of localizations delegates containing this delegate along with
  /// GlobalMaterialLocalizations.delegate, GlobalCupertinoLocalizations.delegate,
  /// and GlobalWidgetsLocalizations.delegate.
  ///
  /// Additional delegates can be added by appending to this list in
  /// MaterialApp. This list does not have to be used at all if a custom list
  /// of delegates is preferred or required.
  static const List<LocalizationsDelegate<dynamic>> localizationsDelegates =
      <LocalizationsDelegate<dynamic>>[
        delegate,
        GlobalMaterialLocalizations.delegate,
        GlobalCupertinoLocalizations.delegate,
        GlobalWidgetsLocalizations.delegate,
      ];

  /// A list of this localizations delegate's supported locales.
  static const List<Locale> supportedLocales = <Locale>[Locale('en'), Locale('en', 'IS')];

  /// Represents a link to a GitHub repository.
  ///
  /// In en, this message translates to:
  /// **'{repoName} GitHub repository'**
  String githubRepo(Object repoName);

  /// A description about how to view the source code for this app.
  ///
  /// In en, this message translates to:
  /// **'To see the source code for this app, please visit the {repoLink}.'**
  String aboutDialogDescription(Object repoLink);

  /// Deselect a (selectable) item
  ///
  /// In en, this message translates to:
  /// **'Deselect'**
  String get deselect;

  /// Indicates the status of a (selectable) item not being selected
  ///
  /// In en, this message translates to:
  /// **'Not selected'**
  String get notSelected;

  /// Select a (selectable) item
  ///
  /// In en, this message translates to:
  /// **'Select'**
  String get select;

  /// Indicates the associated piece of UI is selectable by long pressing it
  ///
  /// In en, this message translates to:
  /// **'Selectable (long press)'**
  String get selectable;

  /// Indicates status of a (selectable) item being selected
  ///
  /// In en, this message translates to:
  /// **'Selected'**
  String get selected;

  /// Sign in label to sign into website.
  ///
  /// In en, this message translates to:
  /// **'SIGN IN'**
  String get signIn;

  /// Password was updated on a different device and the user is required to sign in again
  ///
  /// In en, this message translates to:
  /// **'Your password was updated on your other device. Please sign in again.'**
  String get bannerDemoText;

  /// Show the Banner to the user again.
  ///
  /// In en, this message translates to:
  /// **'Reset the banner'**
  String get bannerDemoResetText;

  /// When the user clicks this button the Banner will toggle multiple actions or a single action
  ///
  /// In en, this message translates to:
  /// **'Multiple actions'**
  String get bannerDemoMultipleText;

  /// If user clicks this button the leading icon in the Banner will disappear
  ///
  /// In en, this message translates to:
  /// **'Leading Icon'**
  String get bannerDemoLeadingText;

  /// When text is pressed the banner widget will be removed from the screen.
  ///
  /// In en, this message translates to:
  /// **'DISMISS'**
  String get dismiss;

  /// Semantic label for back button to exit a study and return to the gallery home page.
  ///
  /// In en, this message translates to:
  /// **'Back to Gallery'**
  String get backToGallery;

  /// Click to see more about the content in the cards demo.
  ///
  /// In en, this message translates to:
  /// **'Explore'**
  String get cardsDemoExplore;

  /// Semantics label for Explore. Label tells user to explore the destinationName to the user. Example Explore Tamil
  ///
  /// In en, this message translates to:
  /// **'Explore {destinationName}'**
  String cardsDemoExploreSemantics(Object destinationName);

  /// Semantics label for Share. Label tells user to share the destinationName to the user. Example Share Tamil
  ///
  /// In en, this message translates to:
  /// **'Share {destinationName}'**
  String cardsDemoShareSemantics(Object destinationName);

  /// The user can tap this button
  ///
  /// In en, this message translates to:
  /// **'Tappable'**
  String get cardsDemoTappable;

  /// The top 10 cities that you can visit in Tamil Nadu
  ///
  /// In en, this message translates to:
  /// **'Top 10 Cities to Visit in Tamil Nadu'**
  String get cardsDemoTravelDestinationTitle1;

  /// Number 10
  ///
  /// In en, this message translates to:
  /// **'Number 10'**
  String get cardsDemoTravelDestinationDescription1;

  /// Thanjavur the city
  ///
  /// In en, this message translates to:
  /// **'Thanjavur'**
  String get cardsDemoTravelDestinationCity1;

  /// Thanjavur, Tamil Nadu is a location
  ///
  /// In en, this message translates to:
  /// **'Thanjavur, Tamil Nadu'**
  String get cardsDemoTravelDestinationLocation1;

  /// Artist that are from Southern India
  ///
  /// In en, this message translates to:
  /// **'Artisans of Southern India'**
  String get cardsDemoTravelDestinationTitle2;

  /// Silk Spinners
  ///
  /// In en, this message translates to:
  /// **'Silk Spinners'**
  String get cardsDemoTravelDestinationDescription2;

  /// Chettinad the city
  ///
  /// In en, this message translates to:
  /// **'Chettinad'**
  String get cardsDemoTravelDestinationCity2;

  /// Sivaganga, Tamil Nadu is a location
  ///
  /// In en, this message translates to:
  /// **'Sivaganga, Tamil Nadu'**
  String get cardsDemoTravelDestinationLocation2;

  /// Brihadisvara Temple
  ///
  /// In en, this message translates to:
  /// **'Brihadisvara Temple'**
  String get cardsDemoTravelDestinationTitle3;

  /// Temples
  ///
  /// In en, this message translates to:
  /// **'Temples'**
  String get cardsDemoTravelDestinationDescription3;

  /// Header title on home screen for Gallery section.
  ///
  /// In en, this message translates to:
  /// **'Gallery'**
  String get homeHeaderGallery;

  /// Header title on home screen for Categories section.
  ///
  /// In en, this message translates to:
  /// **'Categories'**
  String get homeHeaderCategories;

  /// Study description for Shrine.
  ///
  /// In en, this message translates to:
  /// **'A fashionable retail app'**
  String get shrineDescription;

  /// Study description for Fortnightly.
  ///
  /// In en, this message translates to:
  /// **'A content-focused news app'**
  String get fortnightlyDescription;

  /// Study description for Rally.
  ///
  /// In en, this message translates to:
  /// **'A personal finance app'**
  String get rallyDescription;

  /// Study description for Reply.
  ///
  /// In en, this message translates to:
  /// **'An efficient, focused email app'**
  String get replyDescription;

  /// Name for account made up by user.
  ///
  /// In en, this message translates to:
  /// **'Checking'**
  String get rallyAccountDataChecking;

  /// Name for account made up by user.
  ///
  /// In en, this message translates to:
  /// **'Home Savings'**
  String get rallyAccountDataHomeSavings;

  /// Name for account made up by user.
  ///
  /// In en, this message translates to:
  /// **'Car Savings'**
  String get rallyAccountDataCarSavings;

  /// Name for account made up by user.
  ///
  /// In en, this message translates to:
  /// **'Vacation'**
  String get rallyAccountDataVacation;

  /// Title for account statistics. Below a percentage such as 0.10% will be displayed.
  ///
  /// In en, this message translates to:
  /// **'Annual Percentage Yield'**
  String get rallyAccountDetailDataAnnualPercentageYield;

  /// Title for account statistics. Below a dollar amount such as $100 will be displayed.
  ///
  /// In en, this message translates to:
  /// **'Interest Rate'**
  String get rallyAccountDetailDataInterestRate;

  /// Title for account statistics. Below a dollar amount such as $100 will be displayed.
  ///
  /// In en, this message translates to:
  /// **'Interest YTD'**
  String get rallyAccountDetailDataInterestYtd;

  /// Title for account statistics. Below a dollar amount such as $100 will be displayed.
  ///
  /// In en, this message translates to:
  /// **'Interest Paid Last Year'**
  String get rallyAccountDetailDataInterestPaidLastYear;

  /// Title for an account detail. Below a date for when the next account statement is released.
  ///
  /// In en, this message translates to:
  /// **'Next Statement'**
  String get rallyAccountDetailDataNextStatement;

  /// Title for an account detail. Below the name of the account owner will be displayed.
  ///
  /// In en, this message translates to:
  /// **'Account Owner'**
  String get rallyAccountDetailDataAccountOwner;

  /// Title for column where it displays the total dollar amount that the user has in bills.
  ///
  /// In en, this message translates to:
  /// **'Total Amount'**
  String get rallyBillDetailTotalAmount;

  /// Title for column where it displays the amount that the user has paid.
  ///
  /// In en, this message translates to:
  /// **'Amount Paid'**
  String get rallyBillDetailAmountPaid;

  /// Title for column where it displays the amount that the user has due.
  ///
  /// In en, this message translates to:
  /// **'Amount Due'**
  String get rallyBillDetailAmountDue;

  /// Category for budget, to sort expenses / bills in.
  ///
  /// In en, this message translates to:
  /// **'Coffee Shops'**
  String get rallyBudgetCategoryCoffeeShops;

  /// Category for budget, to sort expenses / bills in.
  ///
  /// In en, this message translates to:
  /// **'Groceries'**
  String get rallyBudgetCategoryGroceries;

  /// Category for budget, to sort expenses / bills in.
  ///
  /// In en, this message translates to:
  /// **'Restaurants'**
  String get rallyBudgetCategoryRestaurants;

  /// Category for budget, to sort expenses / bills in.
  ///
  /// In en, this message translates to:
  /// **'Clothing'**
  String get rallyBudgetCategoryClothing;

  /// Title for column where it displays the total dollar cap that the user has for its budget.
  ///
  /// In en, this message translates to:
  /// **'Total Cap'**
  String get rallyBudgetDetailTotalCap;

  /// Title for column where it displays the dollar amount that the user has used in its budget.
  ///
  /// In en, this message translates to:
  /// **'Amount Used'**
  String get rallyBudgetDetailAmountUsed;

  /// Title for column where it displays the dollar amount that the user has left in its budget.
  ///
  /// In en, this message translates to:
  /// **'Amount Left'**
  String get rallyBudgetDetailAmountLeft;

  /// Link to go to the page 'Manage Accounts.
  ///
  /// In en, this message translates to:
  /// **'Manage Accounts'**
  String get rallySettingsManageAccounts;

  /// Link to go to the page 'Tax Documents'.
  ///
  /// In en, this message translates to:
  /// **'Tax Documents'**
  String get rallySettingsTaxDocuments;

  /// Link to go to the page 'Passcode and Touch ID'.
  ///
  /// In en, this message translates to:
  /// **'Passcode and Touch ID'**
  String get rallySettingsPasscodeAndTouchId;

  /// Link to go to the page 'Notifications'.
  ///
  /// In en, this message translates to:
  /// **'Notifications'**
  String get rallySettingsNotifications;

  /// Link to go to the page 'Personal Information'.
  ///
  /// In en, this message translates to:
  /// **'Personal Information'**
  String get rallySettingsPersonalInformation;

  /// Link to go to the page 'Paperless Settings'.
  ///
  /// In en, this message translates to:
  /// **'Paperless Settings'**
  String get rallySettingsPaperlessSettings;

  /// Link to go to the page 'Find ATMs'.
  ///
  /// In en, this message translates to:
  /// **'Find ATMs'**
  String get rallySettingsFindAtms;

  /// Link to go to the page 'Help'.
  ///
  /// In en, this message translates to:
  /// **'Help'**
  String get rallySettingsHelp;

  /// Link to go to the page 'Sign out'.
  ///
  /// In en, this message translates to:
  /// **'Sign out'**
  String get rallySettingsSignOut;

  /// Title for 'total account value' overview page, a dollar value is displayed next to it.
  ///
  /// In en, this message translates to:
  /// **'Total'**
  String get rallyAccountTotal;

  /// Title for 'bills due' page, a dollar value is displayed next to it.
  ///
  /// In en, this message translates to:
  /// **'Due'**
  String get rallyBillsDue;

  /// Title for 'budget left' page, a dollar value is displayed next to it.
  ///
  /// In en, this message translates to:
  /// **'Left'**
  String get rallyBudgetLeft;

  /// Link text for accounts page.
  ///
  /// In en, this message translates to:
  /// **'Accounts'**
  String get rallyAccounts;

  /// Link text for bills page.
  ///
  /// In en, this message translates to:
  /// **'Bills'**
  String get rallyBills;

  /// Link text for budgets page.
  ///
  /// In en, this message translates to:
  /// **'Budgets'**
  String get rallyBudgets;

  /// Title for alerts part of overview page.
  ///
  /// In en, this message translates to:
  /// **'Alerts'**
  String get rallyAlerts;

  /// Link text for button to see all data for category.
  ///
  /// In en, this message translates to:
  /// **'SEE ALL'**
  String get rallySeeAll;

  /// Displayed as 'dollar amount left', for example $46.70 LEFT, for a budget category.
  ///
  /// In en, this message translates to:
  /// **' LEFT'**
  String get rallyFinanceLeft;

  /// The navigation link to the overview page.
  ///
  /// In en, this message translates to:
  /// **'OVERVIEW'**
  String get rallyTitleOverview;

  /// The navigation link to the accounts page.
  ///
  /// In en, this message translates to:
  /// **'ACCOUNTS'**
  String get rallyTitleAccounts;

  /// The navigation link to the bills page.
  ///
  /// In en, this message translates to:
  /// **'BILLS'**
  String get rallyTitleBills;

  /// The navigation link to the budgets page.
  ///
  /// In en, this message translates to:
  /// **'BUDGETS'**
  String get rallyTitleBudgets;

  /// The navigation link to the settings page.
  ///
  /// In en, this message translates to:
  /// **'SETTINGS'**
  String get rallyTitleSettings;

  /// Title for login page for the Rally app (Rally does not need to be translated as it is a product name).
  ///
  /// In en, this message translates to:
  /// **'Login to Rally'**
  String get rallyLoginLoginToRally;

  /// Prompt for signing up for an account.
  ///
  /// In en, this message translates to:
  /// **'Don\'t have an account?'**
  String get rallyLoginNoAccount;

  /// Button text to sign up for an account.
  ///
  /// In en, this message translates to:
  /// **'SIGN UP'**
  String get rallyLoginSignUp;

  /// The username field in an login form.
  ///
  /// In en, this message translates to:
  /// **'Username'**
  String get rallyLoginUsername;

  /// The password field in an login form.
  ///
  /// In en, this message translates to:
  /// **'Password'**
  String get rallyLoginPassword;

  /// The label text to login.
  ///
  /// In en, this message translates to:
  /// **'Login'**
  String get rallyLoginLabelLogin;

  /// Text if the user wants to stay logged in.
  ///
  /// In en, this message translates to:
  /// **'Remember Me'**
  String get rallyLoginRememberMe;

  /// Text for login button.
  ///
  /// In en, this message translates to:
  /// **'LOGIN'**
  String get rallyLoginButtonLogin;

  /// Alert message shown when for example, user has used more than 90% of their shopping budget.
  ///
  /// In en, this message translates to:
  /// **'Heads up, you\'ve used up {percent} of your Shopping budget for this month.'**
  String rallyAlertsMessageHeadsUpShopping(Object percent);

  /// Alert message shown when for example, user has spent $120 on Restaurants this week.
  ///
  /// In en, this message translates to:
  /// **'You\'ve spent {amount} on Restaurants this week.'**
  String rallyAlertsMessageSpentOnRestaurants(Object amount);

  /// Alert message shown when for example, the user has spent $24 in ATM fees this month.
  ///
  /// In en, this message translates to:
  /// **'You\'ve spent {amount} in ATM fees this month'**
  String rallyAlertsMessageATMFees(Object amount);

  /// Alert message shown when for example, the checking account is 1% higher than last month.
  ///
  /// In en, this message translates to:
  /// **'Good work! Your checking account is {percent} higher than last month.'**
  String rallyAlertsMessageCheckingAccount(Object percent);

  /// Alert message shown when you have unassigned transactions.
  ///
  /// In en, this message translates to:
  /// **'{count, plural, =1{Increase your potential tax deduction! Assign categories to 1 unassigned transaction.}other{Increase your potential tax deduction! Assign categories to {count} unassigned transactions.}}'**
  String rallyAlertsMessageUnassignedTransactions(num count);

  /// Semantics label for button to see all accounts. Accounts refer to bank account here.
  ///
  /// In en, this message translates to:
  /// **'See all accounts'**
  String get rallySeeAllAccounts;

  /// Semantics label for button to see all bills.
  ///
  /// In en, this message translates to:
  /// **'See all bills'**
  String get rallySeeAllBills;

  /// Semantics label for button to see all budgets.
  ///
  /// In en, this message translates to:
  /// **'See all budgets'**
  String get rallySeeAllBudgets;

  /// Semantics label for row with bank account name (for example checking) and its bank account number (for example 123), with how much money is deposited in it (for example $12).
  ///
  /// In en, this message translates to:
  /// **'{accountName} account {accountNumber} with {amount}.'**
  String rallyAccountAmount(Object accountName, Object accountNumber, Object amount);

  /// Semantics label for row with a bill (example name is rent), when the bill is due (1/12/2019 for example) and for how much money ($12).
  ///
  /// In en, this message translates to:
  /// **'{billName} bill due {date} for {amount}.'**
  String rallyBillAmount(Object billName, Object date, Object amount);

  /// Semantics label for row with a budget (housing budget for example), with how much is used of the budget (for example $5), the total budget (for example $100) and the amount left in the budget (for example $95).
  ///
  /// In en, this message translates to:
  /// **'{budgetName} budget with {amountUsed} used of {amountTotal}, {amountLeft} left'**
  String rallyBudgetAmount(
    Object budgetName,
    Object amountUsed,
    Object amountTotal,
    Object amountLeft,
  );

  /// Study description for Crane.
  ///
  /// In en, this message translates to:
  /// **'A personalized travel app'**
  String get craneDescription;

  /// Category title on home screen for styles & other demos (for context, the styles demos consist of a color demo and a typography demo).
  ///
  /// In en, this message translates to:
  /// **'STYLES & OTHER'**
  String get homeCategoryReference;

  /// Error message when opening the URL for a demo.
  ///
  /// In en, this message translates to:
  /// **'Couldn\'t display URL:'**
  String get demoInvalidURL;

  /// Tooltip for options button in a demo.
  ///
  /// In en, this message translates to:
  /// **'Options'**
  String get demoOptionsTooltip;

  /// Tooltip for info button in a demo.
  ///
  /// In en, this message translates to:
  /// **'Info'**
  String get demoInfoTooltip;

  /// Tooltip for demo code button in a demo.
  ///
  /// In en, this message translates to:
  /// **'Demo Code'**
  String get demoCodeTooltip;

  /// Tooltip for API documentation button in a demo.
  ///
  /// In en, this message translates to:
  /// **'API Documentation'**
  String get demoDocumentationTooltip;

  /// Tooltip for Full Screen button in a demo.
  ///
  /// In en, this message translates to:
  /// **'Full Screen'**
  String get demoFullscreenTooltip;

  /// Caption for a button to copy all text.
  ///
  /// In en, this message translates to:
  /// **'COPY ALL'**
  String get demoCodeViewerCopyAll;

  /// A message displayed to the user after clicking the COPY ALL button, if the text is successfully copied to the clipboard.
  ///
  /// In en, this message translates to:
  /// **'Copied to clipboard.'**
  String get demoCodeViewerCopiedToClipboardMessage;

  /// A message displayed to the user after clicking the COPY ALL button, if the text CANNOT be copied to the clipboard.
  ///
  /// In en, this message translates to:
  /// **'Failed to copy to clipboard: {error}'**
  String demoCodeViewerFailedToCopyToClipboardMessage(Object error);

  /// Title for an alert that explains what the options button does.
  ///
  /// In en, this message translates to:
  /// **'View options'**
  String get demoOptionsFeatureTitle;

  /// Description for an alert that explains what the options button does.
  ///
  /// In en, this message translates to:
  /// **'Tap here to view available options for this demo.'**
  String get demoOptionsFeatureDescription;

  /// Title for the settings screen.
  ///
  /// In en, this message translates to:
  /// **'Settings'**
  String get settingsTitle;

  /// Accessibility label for the settings button when settings are not showing.
  ///
  /// In en, this message translates to:
  /// **'Settings'**
  String get settingsButtonLabel;

  /// Accessibility label for the settings button when settings are showing.
  ///
  /// In en, this message translates to:
  /// **'Close settings'**
  String get settingsButtonCloseLabel;

  /// Option label to indicate the system default will be used.
  ///
  /// In en, this message translates to:
  /// **'System'**
  String get settingsSystemDefault;

  /// Title for text scaling setting.
  ///
  /// In en, this message translates to:
  /// **'Text scaling'**
  String get settingsTextScaling;

  /// Option label for small text scale setting.
  ///
  /// In en, this message translates to:
  /// **'Small'**
  String get settingsTextScalingSmall;

  /// Option label for normal text scale setting.
  ///
  /// In en, this message translates to:
  /// **'Normal'**
  String get settingsTextScalingNormal;

  /// Option label for large text scale setting.
  ///
  /// In en, this message translates to:
  /// **'Large'**
  String get settingsTextScalingLarge;

  /// Option label for huge text scale setting.
  ///
  /// In en, this message translates to:
  /// **'Huge'**
  String get settingsTextScalingHuge;

  /// Title for text direction setting.
  ///
  /// In en, this message translates to:
  /// **'Text direction'**
  String get settingsTextDirection;

  /// Option label for locale-based text direction setting.
  ///
  /// In en, this message translates to:
  /// **'Based on locale'**
  String get settingsTextDirectionLocaleBased;

  /// Option label for left-to-right text direction setting.
  ///
  /// In en, this message translates to:
  /// **'LTR'**
  String get settingsTextDirectionLTR;

  /// Option label for right-to-left text direction setting.
  ///
  /// In en, this message translates to:
  /// **'RTL'**
  String get settingsTextDirectionRTL;

  /// Title for locale setting.
  ///
  /// In en, this message translates to:
  /// **'Locale'**
  String get settingsLocale;

  /// Title for platform mechanics (iOS, Android, macOS, etc.) setting.
  ///
  /// In en, this message translates to:
  /// **'Platform mechanics'**
  String get settingsPlatformMechanics;

  /// Title for the theme setting.
  ///
  /// In en, this message translates to:
  /// **'Theme'**
  String get settingsTheme;

  /// Title for the dark theme setting.
  ///
  /// In en, this message translates to:
  /// **'Dark'**
  String get settingsDarkTheme;

  /// Title for the light theme setting.
  ///
  /// In en, this message translates to:
  /// **'Light'**
  String get settingsLightTheme;

  /// Title for slow motion setting.
  ///
  /// In en, this message translates to:
  /// **'Slow motion'**
  String get settingsSlowMotion;

  /// Title for information button.
  ///
  /// In en, this message translates to:
  /// **'About Flutter Gallery'**
  String get settingsAbout;

  /// Title for feedback button.
  ///
  /// In en, this message translates to:
  /// **'Send feedback'**
  String get settingsFeedback;

  /// Title for attribution (TOASTER is a proper name and should remain in English).
  ///
  /// In en, this message translates to:
  /// **'Designed by TOASTER in London'**
  String get settingsAttribution;

  /// Title for the material App bar component demo.
  ///
  /// In en, this message translates to:
  /// **'App bar'**
  String get demoAppBarTitle;

  /// Subtitle for the material App bar component demo.
  ///
  /// In en, this message translates to:
  /// **'Displays information and actions relating to the current screen'**
  String get demoAppBarSubtitle;

  /// Description for the material App bar component demo.
  ///
  /// In en, this message translates to:
  /// **'The App bar provides content and actions related to the current screen. It\'s used for branding, screen titles, navigation, and actions'**
  String get demoAppBarDescription;

  /// Title for the material bottom app bar component demo.
  ///
  /// In en, this message translates to:
  /// **'Bottom app bar'**
  String get demoBottomAppBarTitle;

  /// Subtitle for the material bottom app bar component demo.
  ///
  /// In en, this message translates to:
  /// **'Displays navigation and actions at the bottom'**
  String get demoBottomAppBarSubtitle;

  /// Description for the material bottom app bar component demo.
  ///
  /// In en, this message translates to:
  /// **'Bottom app bars provide access to a bottom navigation drawer and up to four actions, including the floating action button.'**
  String get demoBottomAppBarDescription;

  /// A toggle for whether to have a notch (or cutout) in the bottom app bar demo.
  ///
  /// In en, this message translates to:
  /// **'Notch'**
  String get bottomAppBarNotch;

  /// A setting for the position of the floating action button in the bottom app bar demo.
  ///
  /// In en, this message translates to:
  /// **'Floating Action Button Position'**
  String get bottomAppBarPosition;

  /// A setting for the position of the floating action button in the bottom app bar that docks the button in the bar and aligns it at the end.
  ///
  /// In en, this message translates to:
  /// **'Docked - End'**
  String get bottomAppBarPositionDockedEnd;

  /// A setting for the position of the floating action button in the bottom app bar that docks the button in the bar and aligns it in the center.
  ///
  /// In en, this message translates to:
  /// **'Docked - Center'**
  String get bottomAppBarPositionDockedCenter;

  /// A setting for the position of the floating action button in the bottom app bar that places the button above the bar and aligns it at the end.
  ///
  /// In en, this message translates to:
  /// **'Floating - End'**
  String get bottomAppBarPositionFloatingEnd;

  /// A setting for the position of the floating action button in the bottom app bar that places the button above the bar and aligns it in the center.
  ///
  /// In en, this message translates to:
  /// **'Floating - Center'**
  String get bottomAppBarPositionFloatingCenter;

  /// Title for the material banner component demo.
  ///
  /// In en, this message translates to:
  /// **'Banner'**
  String get demoBannerTitle;

  /// Subtitle for the material banner component demo.
  ///
  /// In en, this message translates to:
  /// **'Displaying a banner within a list'**
  String get demoBannerSubtitle;

  /// Description for the material banner component demo.
  ///
  /// In en, this message translates to:
  /// **'A banner displays an important, succinct message, and provides actions for users to address (or dismiss the banner). A user action is required for it to be dismissed.'**
  String get demoBannerDescription;

  /// Title for the material bottom navigation component demo.
  ///
  /// In en, this message translates to:
  /// **'Bottom navigation'**
  String get demoBottomNavigationTitle;

  /// Subtitle for the material bottom navigation component demo.
  ///
  /// In en, this message translates to:
  /// **'Bottom navigation with cross-fading views'**
  String get demoBottomNavigationSubtitle;

  /// Option title for bottom navigation with persistent labels.
  ///
  /// In en, this message translates to:
  /// **'Persistent labels'**
  String get demoBottomNavigationPersistentLabels;

  /// Option title for bottom navigation with only a selected label.
  ///
  /// In en, this message translates to:
  /// **'Selected label'**
  String get demoBottomNavigationSelectedLabel;

  /// Description for the material bottom navigation component demo.
  ///
  /// In en, this message translates to:
  /// **'Bottom navigation bars display three to five destinations at the bottom of a screen. Each destination is represented by an icon and an optional text label. When a bottom navigation icon is tapped, the user is taken to the top-level navigation destination associated with that icon.'**
  String get demoBottomNavigationDescription;

  /// Title for the material buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'Buttons'**
  String get demoButtonTitle;

  /// Subtitle for the material buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'Text, elevated, outlined, and more'**
  String get demoButtonSubtitle;

  /// Title for the text button component demo.
  ///
  /// In en, this message translates to:
  /// **'Text Button'**
  String get demoTextButtonTitle;

  /// Description for the text button component demo.
  ///
  /// In en, this message translates to:
  /// **'A text button displays an ink splash on press but does not lift. Use text buttons on toolbars, in dialogs and inline with padding'**
  String get demoTextButtonDescription;

  /// Title for the elevated button component demo.
  ///
  /// In en, this message translates to:
  /// **'Elevated Button'**
  String get demoElevatedButtonTitle;

  /// Description for the elevated button component demo.
  ///
  /// In en, this message translates to:
  /// **'Elevated buttons add dimension to mostly flat layouts. They emphasize functions on busy or wide spaces.'**
  String get demoElevatedButtonDescription;

  /// Title for the outlined button component demo.
  ///
  /// In en, this message translates to:
  /// **'Outlined Button'**
  String get demoOutlinedButtonTitle;

  /// Description for the outlined button component demo.
  ///
  /// In en, this message translates to:
  /// **'Outlined buttons become opaque and elevate when pressed. They are often paired with raised buttons to indicate an alternative, secondary action.'**
  String get demoOutlinedButtonDescription;

  /// Title for the toggle buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'Toggle Buttons'**
  String get demoToggleButtonTitle;

  /// Description for the toggle buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'Toggle buttons can be used to group related options. To emphasize groups of related toggle buttons, a group should share a common container'**
  String get demoToggleButtonDescription;

  /// Title for the floating action button component demo.
  ///
  /// In en, this message translates to:
  /// **'Floating Action Button'**
  String get demoFloatingButtonTitle;

  /// Description for the floating action button component demo.
  ///
  /// In en, this message translates to:
  /// **'A floating action button is a circular icon button that hovers over content to promote a primary action in the application.'**
  String get demoFloatingButtonDescription;

  /// Title for the material cards component demo.
  ///
  /// In en, this message translates to:
  /// **'Cards'**
  String get demoCardTitle;

  /// Subtitle for the material cards component demo.
  ///
  /// In en, this message translates to:
  /// **'Baseline cards with rounded corners'**
  String get demoCardSubtitle;

  /// Title for the material chips component demo.
  ///
  /// In en, this message translates to:
  /// **'Chips'**
  String get demoChipTitle;

  /// Description for the material cards component demo.
  ///
  /// In en, this message translates to:
  /// **'A card is a sheet of Material used to represent some related information, for example an album, a geographical location, a meal, contact details, etc.'**
  String get demoCardDescription;

  /// Subtitle for the material chips component demo.
  ///
  /// In en, this message translates to:
  /// **'Compact elements that represent an input, attribute, or action'**
  String get demoChipSubtitle;

  /// Title for the action chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Action Chip'**
  String get demoActionChipTitle;

  /// Description for the action chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Action chips are a set of options which trigger an action related to primary content. Action chips should appear dynamically and contextually in a UI.'**
  String get demoActionChipDescription;

  /// Title for the choice chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Choice Chip'**
  String get demoChoiceChipTitle;

  /// Description for the choice chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Choice chips represent a single choice from a set. Choice chips contain related descriptive text or categories.'**
  String get demoChoiceChipDescription;

  /// Title for the filter chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Filter Chip'**
  String get demoFilterChipTitle;

  /// Description for the filter chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Filter chips use tags or descriptive words as a way to filter content.'**
  String get demoFilterChipDescription;

  /// Title for the input chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Input Chip'**
  String get demoInputChipTitle;

  /// Description for the input chip component demo.
  ///
  /// In en, this message translates to:
  /// **'Input chips represent a complex piece of information, such as an entity (person, place, or thing) or conversational text, in a compact form.'**
  String get demoInputChipDescription;

  /// Title for the material data table component demo.
  ///
  /// In en, this message translates to:
  /// **'Data Tables'**
  String get demoDataTableTitle;

  /// Subtitle for the material data table component demo.
  ///
  /// In en, this message translates to:
  /// **'Rows and columns of information'**
  String get demoDataTableSubtitle;

  /// Description for the material data table component demo.
  ///
  /// In en, this message translates to:
  /// **'Data tables display information in a grid-like format of rows and columns. They organize information in a way that\'s easy to scan, so that users can look for patterns and insights.'**
  String get demoDataTableDescription;

  /// Header for the data table component demo about nutrition.
  ///
  /// In en, this message translates to:
  /// **'Nutrition'**
  String get dataTableHeader;

  /// Column header for desserts.
  ///
  /// In en, this message translates to:
  /// **'Dessert (1 serving)'**
  String get dataTableColumnDessert;

  /// Column header for number of calories.
  ///
  /// In en, this message translates to:
  /// **'Calories'**
  String get dataTableColumnCalories;

  /// Column header for number of grams of fat.
  ///
  /// In en, this message translates to:
  /// **'Fat (g)'**
  String get dataTableColumnFat;

  /// Column header for number of grams of carbs.
  ///
  /// In en, this message translates to:
  /// **'Carbs (g)'**
  String get dataTableColumnCarbs;

  /// Column header for number of grams of protein.
  ///
  /// In en, this message translates to:
  /// **'Protein (g)'**
  String get dataTableColumnProtein;

  /// Column header for number of milligrams of sodium.
  ///
  /// In en, this message translates to:
  /// **'Sodium (mg)'**
  String get dataTableColumnSodium;

  /// Column header for daily percentage of calcium.
  ///
  /// In en, this message translates to:
  /// **'Calcium (%)'**
  String get dataTableColumnCalcium;

  /// Column header for daily percentage of iron.
  ///
  /// In en, this message translates to:
  /// **'Iron (%)'**
  String get dataTableColumnIron;

  /// Column row for frozen yogurt.
  ///
  /// In en, this message translates to:
  /// **'Frozen yogurt'**
  String get dataTableRowFrozenYogurt;

  /// Column row for Ice cream sandwich.
  ///
  /// In en, this message translates to:
  /// **'Ice cream sandwich'**
  String get dataTableRowIceCreamSandwich;

  /// Column row for Eclair.
  ///
  /// In en, this message translates to:
  /// **'Eclair'**
  String get dataTableRowEclair;

  /// Column row for Cupcake.
  ///
  /// In en, this message translates to:
  /// **'Cupcake'**
  String get dataTableRowCupcake;

  /// Column row for Gingerbread.
  ///
  /// In en, this message translates to:
  /// **'Gingerbread'**
  String get dataTableRowGingerbread;

  /// Column row for Jelly bean.
  ///
  /// In en, this message translates to:
  /// **'Jelly bean'**
  String get dataTableRowJellyBean;

  /// Column row for Lollipop.
  ///
  /// In en, this message translates to:
  /// **'Lollipop'**
  String get dataTableRowLollipop;

  /// Column row for Honeycomb.
  ///
  /// In en, this message translates to:
  /// **'Honeycomb'**
  String get dataTableRowHoneycomb;

  /// Column row for Donut.
  ///
  /// In en, this message translates to:
  /// **'Donut'**
  String get dataTableRowDonut;

  /// Column row for Apple pie.
  ///
  /// In en, this message translates to:
  /// **'Apple pie'**
  String get dataTableRowApplePie;

  /// A dessert with sugar on it. The parameter is some type of dessert.
  ///
  /// In en, this message translates to:
  /// **'{value} with sugar'**
  String dataTableRowWithSugar(Object value);

  /// A dessert with honey on it. The parameter is some type of dessert.
  ///
  /// In en, this message translates to:
  /// **'{value} with honey'**
  String dataTableRowWithHoney(Object value);

  /// Title for the material dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'Dialogs'**
  String get demoDialogTitle;

  /// Subtitle for the material dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'Simple, alert, and fullscreen'**
  String get demoDialogSubtitle;

  /// Title for the alert dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'Alert'**
  String get demoAlertDialogTitle;

  /// Description for the alert dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'An alert dialog informs the user about situations that require acknowledgement. An alert dialog has an optional title and an optional list of actions.'**
  String get demoAlertDialogDescription;

  /// Title for the alert dialog with title component demo.
  ///
  /// In en, this message translates to:
  /// **'Alert With Title'**
  String get demoAlertTitleDialogTitle;

  /// Title for the simple dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'Simple'**
  String get demoSimpleDialogTitle;

  /// Description for the simple dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'A simple dialog offers the user a choice between several options. A simple dialog has an optional title that is displayed above the choices.'**
  String get demoSimpleDialogDescription;

  /// Title for the divider component demo.
  ///
  /// In en, this message translates to:
  /// **'Divider'**
  String get demoDividerTitle;

  /// Subtitle for the divider component demo.
  ///
  /// In en, this message translates to:
  /// **'A divider is a thin line that groups content in lists and layouts.'**
  String get demoDividerSubtitle;

  /// Description for the divider component demo.
  ///
  /// In en, this message translates to:
  /// **'Dividers can be used in lists, drawers, and elsewhere to separate content.'**
  String get demoDividerDescription;

  /// Title for the vertical divider component demo.
  ///
  /// In en, this message translates to:
  /// **'Vertical Divider'**
  String get demoVerticalDividerTitle;

  /// Title for the grid lists component demo.
  ///
  /// In en, this message translates to:
  /// **'Grid Lists'**
  String get demoGridListsTitle;

  /// Subtitle for the grid lists component demo.
  ///
  /// In en, this message translates to:
  /// **'Row and column layout'**
  String get demoGridListsSubtitle;

  /// Description for the grid lists component demo.
  ///
  /// In en, this message translates to:
  /// **'Grid Lists are best suited for presenting homogeneous data, typically images. Each item in a grid list is called a tile.'**
  String get demoGridListsDescription;

  /// Title for the grid lists image-only component demo.
  ///
  /// In en, this message translates to:
  /// **'Image only'**
  String get demoGridListsImageOnlyTitle;

  /// Title for the grid lists component demo with headers on each tile.
  ///
  /// In en, this message translates to:
  /// **'With header'**
  String get demoGridListsHeaderTitle;

  /// Title for the grid lists component demo with footers on each tile.
  ///
  /// In en, this message translates to:
  /// **'With footer'**
  String get demoGridListsFooterTitle;

  /// Title for the sliders component demo.
  ///
  /// In en, this message translates to:
  /// **'Sliders'**
  String get demoSlidersTitle;

  /// Short description for the sliders component demo.
  ///
  /// In en, this message translates to:
  /// **'Widgets for selecting a value by swiping'**
  String get demoSlidersSubtitle;

  /// Description for the sliders demo.
  ///
  /// In en, this message translates to:
  /// **'Sliders reflect a range of values along a bar, from which users may select a single value. They are ideal for adjusting settings such as volume, brightness, or applying image filters.'**
  String get demoSlidersDescription;

  /// Title for the range sliders component demo.
  ///
  /// In en, this message translates to:
  /// **'Range Sliders'**
  String get demoRangeSlidersTitle;

  /// Description for the range sliders demo.
  ///
  /// In en, this message translates to:
  /// **'Sliders reflect a range of values along a bar. They can have icons on both ends of the bar that reflect a range of values. They are ideal for adjusting settings such as volume, brightness, or applying image filters.'**
  String get demoRangeSlidersDescription;

  /// Title for the custom sliders component demo.
  ///
  /// In en, this message translates to:
  /// **'Custom Sliders'**
  String get demoCustomSlidersTitle;

  /// Description for the custom sliders demo.
  ///
  /// In en, this message translates to:
  /// **'Sliders reflect a range of values along a bar, from which users may select a single value or range of values. The sliders can be themed and customized.'**
  String get demoCustomSlidersDescription;

  /// Text to describe a slider has a continuous value with an editable numerical value.
  ///
  /// In en, this message translates to:
  /// **'Continuous with Editable Numerical Value'**
  String get demoSlidersContinuousWithEditableNumericalValue;

  /// Text to describe that we have a slider with discrete values.
  ///
  /// In en, this message translates to:
  /// **'Discrete'**
  String get demoSlidersDiscrete;

  /// Text to describe that we have a slider with discrete values and a custom theme.
  ///
  /// In en, this message translates to:
  /// **'Discrete Slider with Custom Theme'**
  String get demoSlidersDiscreteSliderWithCustomTheme;

  /// Text to describe that we have a range slider with continuous values and a custom theme.
  ///
  /// In en, this message translates to:
  /// **'Continuous Range Slider with Custom Theme'**
  String get demoSlidersContinuousRangeSliderWithCustomTheme;

  /// Text to describe that we have a slider with continuous values.
  ///
  /// In en, this message translates to:
  /// **'Continuous'**
  String get demoSlidersContinuous;

  /// Label for input field that has an editable numerical value.
  ///
  /// In en, this message translates to:
  /// **'Editable numerical value'**
  String get demoSlidersEditableNumericalValue;

  /// Title for the menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Menu'**
  String get demoMenuTitle;

  /// Title for the context menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Context menu'**
  String get demoContextMenuTitle;

  /// Title for the sectioned menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Sectioned menu'**
  String get demoSectionedMenuTitle;

  /// Title for the simple menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Simple menu'**
  String get demoSimpleMenuTitle;

  /// Title for the checklist menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Checklist menu'**
  String get demoChecklistMenuTitle;

  /// Short description for the menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Menu buttons and simple menus'**
  String get demoMenuSubtitle;

  /// Description for the menu demo.
  ///
  /// In en, this message translates to:
  /// **'A menu displays a list of choices on a temporary surface. They appear when users interact with a button, action, or other control.'**
  String get demoMenuDescription;

  /// The first item in a menu.
  ///
  /// In en, this message translates to:
  /// **'Menu item one'**
  String get demoMenuItemValueOne;

  /// The second item in a menu.
  ///
  /// In en, this message translates to:
  /// **'Menu item two'**
  String get demoMenuItemValueTwo;

  /// The third item in a menu.
  ///
  /// In en, this message translates to:
  /// **'Menu item three'**
  String get demoMenuItemValueThree;

  /// The number one.
  ///
  /// In en, this message translates to:
  /// **'One'**
  String get demoMenuOne;

  /// The number two.
  ///
  /// In en, this message translates to:
  /// **'Two'**
  String get demoMenuTwo;

  /// The number three.
  ///
  /// In en, this message translates to:
  /// **'Three'**
  String get demoMenuThree;

  /// The number four.
  ///
  /// In en, this message translates to:
  /// **'Four'**
  String get demoMenuFour;

  /// Label next to a button that opens a menu. A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'An item with a context menu'**
  String get demoMenuAnItemWithAContextMenuButton;

  /// Text label for a context menu item. A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'Context menu item one'**
  String get demoMenuContextMenuItemOne;

  /// Text label for a disabled menu item. A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'Disabled menu item'**
  String get demoMenuADisabledMenuItem;

  /// Text label for a context menu item three. A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'Context menu item three'**
  String get demoMenuContextMenuItemThree;

  /// Label next to a button that opens a sectioned menu . A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'An item with a sectioned menu'**
  String get demoMenuAnItemWithASectionedMenu;

  /// Button to preview content.
  ///
  /// In en, this message translates to:
  /// **'Preview'**
  String get demoMenuPreview;

  /// Button to share content.
  ///
  /// In en, this message translates to:
  /// **'Share'**
  String get demoMenuShare;

  /// Button to get link for content.
  ///
  /// In en, this message translates to:
  /// **'Get link'**
  String get demoMenuGetLink;

  /// Button to remove content.
  ///
  /// In en, this message translates to:
  /// **'Remove'**
  String get demoMenuRemove;

  /// A text to show what value was selected.
  ///
  /// In en, this message translates to:
  /// **'Selected: {value}'**
  String demoMenuSelected(Object value);

  /// A text to show what value was checked.
  ///
  /// In en, this message translates to:
  /// **'Checked: {value}'**
  String demoMenuChecked(Object value);

  /// Title for the material drawer component demo.
  ///
  /// In en, this message translates to:
  /// **'Navigation Drawer'**
  String get demoNavigationDrawerTitle;

  /// Subtitle for the material drawer component demo.
  ///
  /// In en, this message translates to:
  /// **'Displaying a drawer within appbar'**
  String get demoNavigationDrawerSubtitle;

  /// Description for the material drawer component demo.
  ///
  /// In en, this message translates to:
  /// **'A Material Design panel that slides in horizontally from the edge of the screen to show navigation links in an application.'**
  String get demoNavigationDrawerDescription;

  /// Demo username for navigation drawer.
  ///
  /// In en, this message translates to:
  /// **'User Name'**
  String get demoNavigationDrawerUserName;

  /// Demo email for navigation drawer.
  ///
  /// In en, this message translates to:
  /// **'user.name@example.com'**
  String get demoNavigationDrawerUserEmail;

  /// Drawer Item One.
  ///
  /// In en, this message translates to:
  /// **'Item One'**
  String get demoNavigationDrawerToPageOne;

  /// Drawer Item Two.
  ///
  /// In en, this message translates to:
  /// **'Item Two'**
  String get demoNavigationDrawerToPageTwo;

  /// Description to open navigation drawer.
  ///
  /// In en, this message translates to:
  /// **'Swipe from the edge or tap the upper-left icon to see the drawer'**
  String get demoNavigationDrawerText;

  /// Title for the material Navigation Rail component demo.
  ///
  /// In en, this message translates to:
  /// **'Navigation Rail'**
  String get demoNavigationRailTitle;

  /// Subtitle for the material Navigation Rail component demo.
  ///
  /// In en, this message translates to:
  /// **'Displaying a Navigation Rail within an app'**
  String get demoNavigationRailSubtitle;

  /// Description for the material Navigation Rail component demo.
  ///
  /// In en, this message translates to:
  /// **'A material widget that is meant to be displayed at the left or right of an app to navigate between a small number of views, typically between three and five.'**
  String get demoNavigationRailDescription;

  /// Navigation Rail destination first label.
  ///
  /// In en, this message translates to:
  /// **'First'**
  String get demoNavigationRailFirst;

  /// Navigation Rail destination second label.
  ///
  /// In en, this message translates to:
  /// **'Second'**
  String get demoNavigationRailSecond;

  /// Navigation Rail destination Third label.
  ///
  /// In en, this message translates to:
  /// **'Third'**
  String get demoNavigationRailThird;

  /// Label next to a button that opens a simple menu. A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'An item with a simple menu'**
  String get demoMenuAnItemWithASimpleMenu;

  /// Label next to a button that opens a checklist menu. A menu displays a list of choices on a temporary surface. Used as an example in a demo.
  ///
  /// In en, this message translates to:
  /// **'An item with a checklist menu'**
  String get demoMenuAnItemWithAChecklistMenu;

  /// Title for the fullscreen dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'Fullscreen'**
  String get demoFullscreenDialogTitle;

  /// Description for the fullscreen dialog component demo.
  ///
  /// In en, this message translates to:
  /// **'The fullscreenDialog property specifies whether the incoming page is a fullscreen modal dialog'**
  String get demoFullscreenDialogDescription;

  /// Title for the cupertino activity indicator component demo.
  ///
  /// In en, this message translates to:
  /// **'Activity indicator'**
  String get demoCupertinoActivityIndicatorTitle;

  /// Subtitle for the cupertino activity indicator component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style activity indicators'**
  String get demoCupertinoActivityIndicatorSubtitle;

  /// Description for the cupertino activity indicator component demo.
  ///
  /// In en, this message translates to:
  /// **'An iOS-style activity indicator that spins clockwise.'**
  String get demoCupertinoActivityIndicatorDescription;

  /// Title for the cupertino buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'Buttons'**
  String get demoCupertinoButtonsTitle;

  /// Subtitle for the cupertino buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style buttons'**
  String get demoCupertinoButtonsSubtitle;

  /// Description for the cupertino buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'An iOS-style button. It takes in text and/or an icon that fades out and in on touch. May optionally have a background.'**
  String get demoCupertinoButtonsDescription;

  /// Title for the cupertino context menu component demo.
  ///
  /// In en, this message translates to:
  /// **'Context Menu'**
  String get demoCupertinoContextMenuTitle;

  /// Subtitle for the cupertino context menu component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style context menu'**
  String get demoCupertinoContextMenuSubtitle;

  /// Description for the cupertino context menu component demo.
  ///
  /// In en, this message translates to:
  /// **'An iOS-style full screen contextual menu that appears when an element is long-pressed.'**
  String get demoCupertinoContextMenuDescription;

  /// Context menu list item one
  ///
  /// In en, this message translates to:
  /// **'Action one'**
  String get demoCupertinoContextMenuActionOne;

  /// Context menu list item two
  ///
  /// In en, this message translates to:
  /// **'Action two'**
  String get demoCupertinoContextMenuActionTwo;

  /// Context menu text.
  ///
  /// In en, this message translates to:
  /// **'Tap and hold the Flutter logo to see the context menu.'**
  String get demoCupertinoContextMenuActionText;

  /// Title for the cupertino alerts component demo.
  ///
  /// In en, this message translates to:
  /// **'Alerts'**
  String get demoCupertinoAlertsTitle;

  /// Subtitle for the cupertino alerts component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style alert dialogs'**
  String get demoCupertinoAlertsSubtitle;

  /// Title for the cupertino alert component demo.
  ///
  /// In en, this message translates to:
  /// **'Alert'**
  String get demoCupertinoAlertTitle;

  /// Description for the cupertino alert component demo.
  ///
  /// In en, this message translates to:
  /// **'An alert dialog informs the user about situations that require acknowledgement. An alert dialog has an optional title, optional content, and an optional list of actions. The title is displayed above the content and the actions are displayed below the content.'**
  String get demoCupertinoAlertDescription;

  /// Title for the cupertino alert with title component demo.
  ///
  /// In en, this message translates to:
  /// **'Alert With Title'**
  String get demoCupertinoAlertWithTitleTitle;

  /// Title for the cupertino alert with buttons component demo.
  ///
  /// In en, this message translates to:
  /// **'Alert With Buttons'**
  String get demoCupertinoAlertButtonsTitle;

  /// Title for the cupertino alert buttons only component demo.
  ///
  /// In en, this message translates to:
  /// **'Alert Buttons Only'**
  String get demoCupertinoAlertButtonsOnlyTitle;

  /// Title for the cupertino action sheet component demo.
  ///
  /// In en, this message translates to:
  /// **'Action Sheet'**
  String get demoCupertinoActionSheetTitle;

  /// Description for the cupertino action sheet component demo.
  ///
  /// In en, this message translates to:
  /// **'An action sheet is a specific style of alert that presents the user with a set of two or more choices related to the current context. An action sheet can have a title, an additional message, and a list of actions.'**
  String get demoCupertinoActionSheetDescription;

  /// Title for the cupertino navigation bar component demo.
  ///
  /// In en, this message translates to:
  /// **'Navigation bar'**
  String get demoCupertinoNavigationBarTitle;

  /// Subtitle for the cupertino navigation bar component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style navigation bar'**
  String get demoCupertinoNavigationBarSubtitle;

  /// Description for the cupertino navigation bar component demo.
  ///
  /// In en, this message translates to:
  /// **'An iOS-styled navigation bar. The navigation bar is a toolbar that minimally consists of a page title, in the middle of the toolbar.'**
  String get demoCupertinoNavigationBarDescription;

  /// Title for the cupertino pickers component demo.
  ///
  /// In en, this message translates to:
  /// **'Pickers'**
  String get demoCupertinoPickerTitle;

  /// Subtitle for the cupertino pickers component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style pickers'**
  String get demoCupertinoPickerSubtitle;

  /// Description for the cupertino pickers component demo.
  ///
  /// In en, this message translates to:
  /// **'An iOS-style picker widget that can be used to select strings, dates, times, or both date and time.'**
  String get demoCupertinoPickerDescription;

  /// Label to open a countdown timer picker.
  ///
  /// In en, this message translates to:
  /// **'Timer'**
  String get demoCupertinoPickerTimer;

  /// Label to open an iOS picker.
  ///
  /// In en, this message translates to:
  /// **'Picker'**
  String get demoCupertinoPicker;

  /// Label to open a date picker.
  ///
  /// In en, this message translates to:
  /// **'Date'**
  String get demoCupertinoPickerDate;

  /// Label to open a time picker.
  ///
  /// In en, this message translates to:
  /// **'Time'**
  String get demoCupertinoPickerTime;

  /// Label to open a date and time picker.
  ///
  /// In en, this message translates to:
  /// **'Date and Time'**
  String get demoCupertinoPickerDateTime;

  /// Title for the cupertino pull-to-refresh component demo.
  ///
  /// In en, this message translates to:
  /// **'Pull to refresh'**
  String get demoCupertinoPullToRefreshTitle;

  /// Subtitle for the cupertino pull-to-refresh component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style pull to refresh control'**
  String get demoCupertinoPullToRefreshSubtitle;

  /// Description for the cupertino pull-to-refresh component demo.
  ///
  /// In en, this message translates to:
  /// **'A widget implementing the iOS-style pull to refresh content control.'**
  String get demoCupertinoPullToRefreshDescription;

  /// Title for the cupertino segmented control component demo.
  ///
  /// In en, this message translates to:
  /// **'Segmented control'**
  String get demoCupertinoSegmentedControlTitle;

  /// Subtitle for the cupertino segmented control component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style segmented control'**
  String get demoCupertinoSegmentedControlSubtitle;

  /// Description for the cupertino segmented control component demo.
  ///
  /// In en, this message translates to:
  /// **'Used to select between a number of mutually exclusive options. When one option in the segmented control is selected, the other options in the segmented control cease to be selected.'**
  String get demoCupertinoSegmentedControlDescription;

  /// Title for the cupertino slider component demo.
  ///
  /// In en, this message translates to:
  /// **'Slider'**
  String get demoCupertinoSliderTitle;

  /// Subtitle for the cupertino slider component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style slider'**
  String get demoCupertinoSliderSubtitle;

  /// Description for the cupertino slider component demo.
  ///
  /// In en, this message translates to:
  /// **'A slider can be used to select from either a continuous or a discrete set of values.'**
  String get demoCupertinoSliderDescription;

  /// A label for a continuous slider that indicates what value it is set to.
  ///
  /// In en, this message translates to:
  /// **'Continuous: {value}'**
  String demoCupertinoSliderContinuous(Object value);

  /// A label for a discrete slider that indicates what value it is set to.
  ///
  /// In en, this message translates to:
  /// **'Discrete: {value}'**
  String demoCupertinoSliderDiscrete(Object value);

  /// Subtitle for the cupertino switch component demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style switch'**
  String get demoCupertinoSwitchSubtitle;

  /// Description for the cupertino switch component demo.
  ///
  /// In en, this message translates to:
  /// **'A switch is used to toggle the on/off state of a single setting.'**
  String get demoCupertinoSwitchDescription;

  /// Title for the cupertino bottom tab bar demo.
  ///
  /// In en, this message translates to:
  /// **'Tab bar'**
  String get demoCupertinoTabBarTitle;

  /// Subtitle for the cupertino bottom tab bar demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style bottom tab bar'**
  String get demoCupertinoTabBarSubtitle;

  /// Description for the cupertino bottom tab bar demo.
  ///
  /// In en, this message translates to:
  /// **'An iOS-style bottom navigation tab bar. Displays multiple tabs with one tab being active, the first tab by default.'**
  String get demoCupertinoTabBarDescription;

  /// Title for the home tab in the bottom tab bar demo.
  ///
  /// In en, this message translates to:
  /// **'Home'**
  String get cupertinoTabBarHomeTab;

  /// Title for the chat tab in the bottom tab bar demo.
  ///
  /// In en, this message translates to:
  /// **'Chat'**
  String get cupertinoTabBarChatTab;

  /// Title for the profile tab in the bottom tab bar demo.
  ///
  /// In en, this message translates to:
  /// **'Profile'**
  String get cupertinoTabBarProfileTab;

  /// Title for the cupertino text field demo.
  ///
  /// In en, this message translates to:
  /// **'Text fields'**
  String get demoCupertinoTextFieldTitle;

  /// Subtitle for the cupertino text field demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style text fields'**
  String get demoCupertinoTextFieldSubtitle;

  /// Description for the cupertino text field demo.
  ///
  /// In en, this message translates to:
  /// **'A text field lets the user enter text, either with a hardware keyboard or with an onscreen keyboard.'**
  String get demoCupertinoTextFieldDescription;

  /// The placeholder for a text field where a user would enter their PIN number.
  ///
  /// In en, this message translates to:
  /// **'PIN'**
  String get demoCupertinoTextFieldPIN;

  /// Title for the cupertino search text field demo.
  ///
  /// In en, this message translates to:
  /// **'Search text field'**
  String get demoCupertinoSearchTextFieldTitle;

  /// Subtitle for the cupertino search text field demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style search text field'**
  String get demoCupertinoSearchTextFieldSubtitle;

  /// Description for the cupertino search text field demo.
  ///
  /// In en, this message translates to:
  /// **'A search text field that lets the user search by entering text, and that can offer and filter suggestions.'**
  String get demoCupertinoSearchTextFieldDescription;

  /// The placeholder for a search text field demo.
  ///
  /// In en, this message translates to:
  /// **'Enter some text'**
  String get demoCupertinoSearchTextFieldPlaceholder;

  /// Title for the cupertino scrollbar demo.
  ///
  /// In en, this message translates to:
  /// **'Scrollbar'**
  String get demoCupertinoScrollbarTitle;

  /// Subtitle for the cupertino scrollbar demo.
  ///
  /// In en, this message translates to:
  /// **'iOS-style scrollbar'**
  String get demoCupertinoScrollbarSubtitle;

  /// Description for the cupertino scrollbar demo.
  ///
  /// In en, this message translates to:
  /// **'A scrollbar that wraps the given child'**
  String get demoCupertinoScrollbarDescription;

  /// Title for the motion demo.
  ///
  /// In en, this message translates to:
  /// **'Motion'**
  String get demoMotionTitle;

  /// Subtitle for the motion demo.
  ///
  /// In en, this message translates to:
  /// **'All of the predefined transition patterns'**
  String get demoMotionSubtitle;

  /// Instructions for the container transform demo located in the app bar.
  ///
  /// In en, this message translates to:
  /// **'Cards, Lists & FAB'**
  String get demoContainerTransformDemoInstructions;

  /// Instructions for the shared x axis demo located in the app bar.
  ///
  /// In en, this message translates to:
  /// **'Next and Back Buttons'**
  String get demoSharedXAxisDemoInstructions;

  /// Instructions for the shared y axis demo located in the app bar.
  ///
  /// In en, this message translates to:
  /// **'Sort by \"Recently Played\"'**
  String get demoSharedYAxisDemoInstructions;

  /// Instructions for the shared z axis demo located in the app bar.
  ///
  /// In en, this message translates to:
  /// **'Settings icon button'**
  String get demoSharedZAxisDemoInstructions;

  /// Instructions for the fade through demo located in the app bar.
  ///
  /// In en, this message translates to:
  /// **'Bottom navigation'**
  String get demoFadeThroughDemoInstructions;

  /// Instructions for the fade scale demo located in the app bar.
  ///
  /// In en, this message translates to:
  /// **'Modal and FAB'**
  String get demoFadeScaleDemoInstructions;

  /// Title for the container transform demo.
  ///
  /// In en, this message translates to:
  /// **'Container Transform'**
  String get demoContainerTransformTitle;

  /// Description for the container transform demo.
  ///
  /// In en, this message translates to:
  /// **'The container transform pattern is designed for transitions between UI elements that include a container. This pattern creates a visible connection between two UI elements'**
  String get demoContainerTransformDescription;

  /// Title for the container transform modal bottom sheet.
  ///
  /// In en, this message translates to:
  /// **'Fade mode'**
  String get demoContainerTransformModalBottomSheetTitle;

  /// Description for container transform fade type setting.
  ///
  /// In en, this message translates to:
  /// **'FADE'**
  String get demoContainerTransformTypeFade;

  /// Description for container transform fade through type setting.
  ///
  /// In en, this message translates to:
  /// **'FADE THROUGH'**
  String get demoContainerTransformTypeFadeThrough;

  /// The placeholder for the motion demos title properties.
  ///
  /// In en, this message translates to:
  /// **'Title'**
  String get demoMotionPlaceholderTitle;

  /// The placeholder for the motion demos subtitle properties.
  ///
  /// In en, this message translates to:
  /// **'Secondary text'**
  String get demoMotionPlaceholderSubtitle;

  /// The placeholder for the motion demos shortened subtitle properties.
  ///
  /// In en, this message translates to:
  /// **'Secondary'**
  String get demoMotionSmallPlaceholderSubtitle;

  /// The title for the details page in the motion demos.
  ///
  /// In en, this message translates to:
  /// **'Details Page'**
  String get demoMotionDetailsPageTitle;

  /// The title for a list tile in the motion demos.
  ///
  /// In en, this message translates to:
  /// **'List item'**
  String get demoMotionListTileTitle;

  /// Description for the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'The shared axis pattern is used for transitions between the UI elements that have a spatial or navigational relationship. This pattern uses a shared transformation on the x, y, or z axis to reinforce the relationship between elements.'**
  String get demoSharedAxisDescription;

  /// Title for the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Shared x-axis'**
  String get demoSharedXAxisTitle;

  /// Button text for back button in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'BACK'**
  String get demoSharedXAxisBackButtonText;

  /// Button text for the next button in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'NEXT'**
  String get demoSharedXAxisNextButtonText;

  /// Title for course selection page in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Streamline your courses'**
  String get demoSharedXAxisCoursePageTitle;

  /// Subtitle for course selection page in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Bundled categories appear as groups in your feed. You can always change this later.'**
  String get demoSharedXAxisCoursePageSubtitle;

  /// Title for the Arts & Crafts course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Arts & Crafts'**
  String get demoSharedXAxisArtsAndCraftsCourseTitle;

  /// Title for the Business course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Business'**
  String get demoSharedXAxisBusinessCourseTitle;

  /// Title for the Illustration course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Illustration'**
  String get demoSharedXAxisIllustrationCourseTitle;

  /// Title for the Design course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Design'**
  String get demoSharedXAxisDesignCourseTitle;

  /// Title for the Culinary course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Culinary'**
  String get demoSharedXAxisCulinaryCourseTitle;

  /// Subtitle for a bundled course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Bundled'**
  String get demoSharedXAxisBundledCourseSubtitle;

  /// Subtitle for a individual course in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Shown Individually'**
  String get demoSharedXAxisIndividualCourseSubtitle;

  /// Welcome text for sign in page in the shared x axis demo. David Park is a name and does not need to be translated.
  ///
  /// In en, this message translates to:
  /// **'Hi David Park'**
  String get demoSharedXAxisSignInWelcomeText;

  /// Subtitle text for sign in page in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Sign in with your account'**
  String get demoSharedXAxisSignInSubtitleText;

  /// Label text for the sign in text field in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'Email or phone number'**
  String get demoSharedXAxisSignInTextFieldLabel;

  /// Button text for the forgot email button in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'FORGOT EMAIL?'**
  String get demoSharedXAxisForgotEmailButtonText;

  /// Button text for the create account button in the shared x axis demo.
  ///
  /// In en, this message translates to:
  /// **'CREATE ACCOUNT'**
  String get demoSharedXAxisCreateAccountButtonText;

  /// Title for the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'Shared y-axis'**
  String get demoSharedYAxisTitle;

  /// Text for album count in the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'268 albums'**
  String get demoSharedYAxisAlbumCount;

  /// Title for alphabetical sorting type in the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'A-Z'**
  String get demoSharedYAxisAlphabeticalSortTitle;

  /// Title for recently played sorting type in the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'Recently played'**
  String get demoSharedYAxisRecentSortTitle;

  /// Title for an AlbumTile in the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'Album'**
  String get demoSharedYAxisAlbumTileTitle;

  /// Subtitle for an AlbumTile in the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'Artist'**
  String get demoSharedYAxisAlbumTileSubtitle;

  /// Duration unit for an AlbumTile in the shared y axis demo.
  ///
  /// In en, this message translates to:
  /// **'min'**
  String get demoSharedYAxisAlbumTileDurationUnit;

  /// Title for the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Shared z-axis'**
  String get demoSharedZAxisTitle;

  /// Title for the settings page in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Settings'**
  String get demoSharedZAxisSettingsPageTitle;

  /// Title for burger recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Burger'**
  String get demoSharedZAxisBurgerRecipeTitle;

  /// Subtitle for the burger recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Burger recipe'**
  String get demoSharedZAxisBurgerRecipeDescription;

  /// Title for sandwich recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Sandwich'**
  String get demoSharedZAxisSandwichRecipeTitle;

  /// Subtitle for the sandwich recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Sandwich recipe'**
  String get demoSharedZAxisSandwichRecipeDescription;

  /// Title for dessert recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Dessert'**
  String get demoSharedZAxisDessertRecipeTitle;

  /// Subtitle for the dessert recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Dessert recipe'**
  String get demoSharedZAxisDessertRecipeDescription;

  /// Title for shrimp plate recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Shrimp'**
  String get demoSharedZAxisShrimpPlateRecipeTitle;

  /// Subtitle for the shrimp plate recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Shrimp plate recipe'**
  String get demoSharedZAxisShrimpPlateRecipeDescription;

  /// Title for crab plate recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Crab'**
  String get demoSharedZAxisCrabPlateRecipeTitle;

  /// Subtitle for the crab plate recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Crab plate recipe'**
  String get demoSharedZAxisCrabPlateRecipeDescription;

  /// Title for beef sandwich recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Beef Sandwich'**
  String get demoSharedZAxisBeefSandwichRecipeTitle;

  /// Subtitle for the beef sandwich recipe tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Beef Sandwich recipe'**
  String get demoSharedZAxisBeefSandwichRecipeDescription;

  /// Title for list of saved recipes in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Saved Recipes'**
  String get demoSharedZAxisSavedRecipesListTitle;

  /// Text label for profile setting tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Profile'**
  String get demoSharedZAxisProfileSettingLabel;

  /// Text label for notifications setting tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Notifications'**
  String get demoSharedZAxisNotificationSettingLabel;

  /// Text label for the privacy setting tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Privacy'**
  String get demoSharedZAxisPrivacySettingLabel;

  /// Text label for the help setting tile in the shared z axis demo.
  ///
  /// In en, this message translates to:
  /// **'Help'**
  String get demoSharedZAxisHelpSettingLabel;

  /// Title for the fade through demo.
  ///
  /// In en, this message translates to:
  /// **'Fade through'**
  String get demoFadeThroughTitle;

  /// Description for the fade through demo.
  ///
  /// In en, this message translates to:
  /// **'The fade through pattern is used for transitions between UI elements that do not have a strong relationship to each other.'**
  String get demoFadeThroughDescription;

  /// Text for albums bottom navigation bar destination in the fade through demo.
  ///
  /// In en, this message translates to:
  /// **'Albums'**
  String get demoFadeThroughAlbumsDestination;

  /// Text for photos bottom navigation bar destination in the fade through demo.
  ///
  /// In en, this message translates to:
  /// **'Photos'**
  String get demoFadeThroughPhotosDestination;

  /// Text for search bottom navigation bar destination in the fade through demo.
  ///
  /// In en, this message translates to:
  /// **'Search'**
  String get demoFadeThroughSearchDestination;

  /// Placeholder for example card title in the fade through demo.
  ///
  /// In en, this message translates to:
  /// **'123 photos'**
  String get demoFadeThroughTextPlaceholder;

  /// Title for the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'Fade'**
  String get demoFadeScaleTitle;

  /// Description for the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'The fade pattern is used for UI elements that enter or exit within the bounds of the screen, such as a dialog that fades in the center of the screen.'**
  String get demoFadeScaleDescription;

  /// Button text to show alert dialog in the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'SHOW MODAL'**
  String get demoFadeScaleShowAlertDialogButton;

  /// Button text to show fab in the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'SHOW FAB'**
  String get demoFadeScaleShowFabButton;

  /// Button text to hide fab in the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'HIDE FAB'**
  String get demoFadeScaleHideFabButton;

  /// Generic header for alert dialog in the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'Alert Dialog'**
  String get demoFadeScaleAlertDialogHeader;

  /// Button text for alert dialog cancel button in the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'CANCEL'**
  String get demoFadeScaleAlertDialogCancelButton;

  /// Button text for alert dialog discard button in the fade scale demo.
  ///
  /// In en, this message translates to:
  /// **'DISCARD'**
  String get demoFadeScaleAlertDialogDiscardButton;

  /// Title for the colors demo.
  ///
  /// In en, this message translates to:
  /// **'Colors'**
  String get demoColorsTitle;

  /// Subtitle for the colors demo.
  ///
  /// In en, this message translates to:
  /// **'All of the predefined colors'**
  String get demoColorsSubtitle;

  /// Description for the colors demo. Material Design should remain capitalized.
  ///
  /// In en, this message translates to:
  /// **'Color and color swatch constants which represent Material Design\'s color palette.'**
  String get demoColorsDescription;

  /// Title for the typography demo.
  ///
  /// In en, this message translates to:
  /// **'Typography'**
  String get demoTypographyTitle;

  /// Subtitle for the typography demo.
  ///
  /// In en, this message translates to:
  /// **'All of the predefined text styles'**
  String get demoTypographySubtitle;

  /// Description for the typography demo. Material Design should remain capitalized.
  ///
  /// In en, this message translates to:
  /// **'Definitions for the various typographical styles found in Material Design.'**
  String get demoTypographyDescription;

  /// Title for the 2D transformations demo.
  ///
  /// In en, this message translates to:
  /// **'2D transformations'**
  String get demo2dTransformationsTitle;

  /// Subtitle for the 2D transformations demo.
  ///
  /// In en, this message translates to:
  /// **'Pan and zoom'**
  String get demo2dTransformationsSubtitle;

  /// Description for the 2D transformations demo.
  ///
  /// In en, this message translates to:
  /// **'Tap to edit tiles, and use gestures to move around the scene. Drag to pan and pinch with two fingers to zoom. Press the reset button to return to the starting orientation.'**
  String get demo2dTransformationsDescription;

  /// Tooltip for a button to reset the transformations (scale, translation) for the 2D transformations demo.
  ///
  /// In en, this message translates to:
  /// **'Reset transformations'**
  String get demo2dTransformationsResetTooltip;

  /// Tooltip for a button to edit a tile.
  ///
  /// In en, this message translates to:
  /// **'Edit tile'**
  String get demo2dTransformationsEditTooltip;

  /// Text for a generic button.
  ///
  /// In en, this message translates to:
  /// **'BUTTON'**
  String get buttonText;

  /// Title for bottom sheet demo.
  ///
  /// In en, this message translates to:
  /// **'Bottom sheet'**
  String get demoBottomSheetTitle;

  /// Description for bottom sheet demo.
  ///
  /// In en, this message translates to:
  /// **'Persistent and modal bottom sheets'**
  String get demoBottomSheetSubtitle;

  /// Title for persistent bottom sheet demo.
  ///
  /// In en, this message translates to:
  /// **'Persistent bottom sheet'**
  String get demoBottomSheetPersistentTitle;

  /// Description for persistent bottom sheet demo.
  ///
  /// In en, this message translates to:
  /// **'A persistent bottom sheet shows information that supplements the primary content of the app. A persistent bottom sheet remains visible even when the user interacts with other parts of the app.'**
  String get demoBottomSheetPersistentDescription;

  /// Title for modal bottom sheet demo.
  ///
  /// In en, this message translates to:
  /// **'Modal bottom sheet'**
  String get demoBottomSheetModalTitle;

  /// Description for modal bottom sheet demo.
  ///
  /// In en, this message translates to:
  /// **'A modal bottom sheet is an alternative to a menu or a dialog and prevents the user from interacting with the rest of the app.'**
  String get demoBottomSheetModalDescription;

  /// Semantic label for add icon.
  ///
  /// In en, this message translates to:
  /// **'Add'**
  String get demoBottomSheetAddLabel;

  /// Button text to show bottom sheet.
  ///
  /// In en, this message translates to:
  /// **'SHOW BOTTOM SHEET'**
  String get demoBottomSheetButtonText;

  /// Generic header placeholder.
  ///
  /// In en, this message translates to:
  /// **'Header'**
  String get demoBottomSheetHeader;

  /// Generic item placeholder.
  ///
  /// In en, this message translates to:
  /// **'Item {value}'**
  String demoBottomSheetItem(Object value);

  /// Title for lists demo.
  ///
  /// In en, this message translates to:
  /// **'Lists'**
  String get demoListsTitle;

  /// Subtitle for lists demo.
  ///
  /// In en, this message translates to:
  /// **'Scrolling list layouts'**
  String get demoListsSubtitle;

  /// Description for lists demo. This describes what a single row in a list consists of.
  ///
  /// In en, this message translates to:
  /// **'A single fixed-height row that typically contains some text as well as a leading or trailing icon.'**
  String get demoListsDescription;

  /// Title for lists demo with only one line of text per row.
  ///
  /// In en, this message translates to:
  /// **'One Line'**
  String get demoOneLineListsTitle;

  /// Title for lists demo with two lines of text per row.
  ///
  /// In en, this message translates to:
  /// **'Two Lines'**
  String get demoTwoLineListsTitle;

  /// Text that appears in the second line of a list item.
  ///
  /// In en, this message translates to:
  /// **'Secondary text'**
  String get demoListsSecondary;

  /// Title for progress indicators demo.
  ///
  /// In en, this message translates to:
  /// **'Progress indicators'**
  String get demoProgressIndicatorTitle;

  /// Subtitle for progress indicators demo.
  ///
  /// In en, this message translates to:
  /// **'Linear, circular, indeterminate'**
  String get demoProgressIndicatorSubtitle;

  /// Title for circular progress indicator demo.
  ///
  /// In en, this message translates to:
  /// **'Circular Progress Indicator'**
  String get demoCircularProgressIndicatorTitle;

  /// Description for circular progress indicator demo.
  ///
  /// In en, this message translates to:
  /// **'A Material Design circular progress indicator, which spins to indicate that the application is busy.'**
  String get demoCircularProgressIndicatorDescription;

  /// Title for linear progress indicator demo.
  ///
  /// In en, this message translates to:
  /// **'Linear Progress Indicator'**
  String get demoLinearProgressIndicatorTitle;

  /// Description for linear progress indicator demo.
  ///
  /// In en, this message translates to:
  /// **'A Material Design linear progress indicator, also known as a progress bar.'**
  String get demoLinearProgressIndicatorDescription;

  /// Title for pickers demo.
  ///
  /// In en, this message translates to:
  /// **'Pickers'**
  String get demoPickersTitle;

  /// Subtitle for pickers demo.
  ///
  /// In en, this message translates to:
  /// **'Date and time selection'**
  String get demoPickersSubtitle;

  /// Title for date picker demo.
  ///
  /// In en, this message translates to:
  /// **'Date Picker'**
  String get demoDatePickerTitle;

  /// Description for date picker demo.
  ///
  /// In en, this message translates to:
  /// **'Shows a dialog containing a Material Design date picker.'**
  String get demoDatePickerDescription;

  /// Title for time picker demo.
  ///
  /// In en, this message translates to:
  /// **'Time Picker'**
  String get demoTimePickerTitle;

  /// Description for time picker demo.
  ///
  /// In en, this message translates to:
  /// **'Shows a dialog containing a Material Design time picker.'**
  String get demoTimePickerDescription;

  /// Title for date range picker demo.
  ///
  /// In en, this message translates to:
  /// **'Date Range Picker'**
  String get demoDateRangePickerTitle;

  /// Description for date range picker demo.
  ///
  /// In en, this message translates to:
  /// **'Shows a dialog containing a Material Design date range picker.'**
  String get demoDateRangePickerDescription;

  /// Button text to show the date or time picker in the demo.
  ///
  /// In en, this message translates to:
  /// **'SHOW PICKER'**
  String get demoPickersShowPicker;

  /// Title for tabs demo.
  ///
  /// In en, this message translates to:
  /// **'Tabs'**
  String get demoTabsTitle;

  /// Title for tabs demo with a tab bar that scrolls.
  ///
  /// In en, this message translates to:
  /// **'Scrolling'**
  String get demoTabsScrollingTitle;

  /// Title for tabs demo with a tab bar that doesn't scroll.
  ///
  /// In en, this message translates to:
  /// **'Non-scrolling'**
  String get demoTabsNonScrollingTitle;

  /// Subtitle for tabs demo.
  ///
  /// In en, this message translates to:
  /// **'Tabs with independently scrollable views'**
  String get demoTabsSubtitle;

  /// Description for tabs demo.
  ///
  /// In en, this message translates to:
  /// **'Tabs organize content across different screens, data sets, and other interactions.'**
  String get demoTabsDescription;

  /// Title for snackbars demo.
  ///
  /// In en, this message translates to:
  /// **'Snackbars'**
  String get demoSnackbarsTitle;

  /// Subtitle for snackbars demo.
  ///
  /// In en, this message translates to:
  /// **'Snackbars show messages at the bottom of the screen'**
  String get demoSnackbarsSubtitle;

  /// Description for snackbars demo.
  ///
  /// In en, this message translates to:
  /// **'Snackbars inform users of a process that an app has performed or will perform. They appear temporarily, towards the bottom of the screen. They shouldn\'t interrupt the user experience, and they don\'t require user input to disappear.'**
  String get demoSnackbarsDescription;

  /// Label for button to show a snackbar.
  ///
  /// In en, this message translates to:
  /// **'SHOW A SNACKBAR'**
  String get demoSnackbarsButtonLabel;

  /// Text to show on a snackbar.
  ///
  /// In en, this message translates to:
  /// **'This is a snackbar.'**
  String get demoSnackbarsText;

  /// Label for action button text on the snackbar.
  ///
  /// In en, this message translates to:
  /// **'ACTION'**
  String get demoSnackbarsActionButtonLabel;

  /// Text that appears when you press on a snackbars' action.
  ///
  /// In en, this message translates to:
  /// **'You pressed the snackbar action.'**
  String get demoSnackbarsAction;

  /// Title for selection controls demo.
  ///
  /// In en, this message translates to:
  /// **'Selection controls'**
  String get demoSelectionControlsTitle;

  /// Subtitle for selection controls demo.
  ///
  /// In en, this message translates to:
  /// **'Checkboxes, radio buttons, and switches'**
  String get demoSelectionControlsSubtitle;

  /// Title for the checkbox (selection controls) demo.
  ///
  /// In en, this message translates to:
  /// **'Checkbox'**
  String get demoSelectionControlsCheckboxTitle;

  /// Description for the checkbox (selection controls) demo.
  ///
  /// In en, this message translates to:
  /// **'Checkboxes allow the user to select multiple options from a set. A normal checkbox\'s value is true or false and a tristate checkbox\'s value can also be null.'**
  String get demoSelectionControlsCheckboxDescription;

  /// Title for the radio button (selection controls) demo.
  ///
  /// In en, this message translates to:
  /// **'Radio'**
  String get demoSelectionControlsRadioTitle;

  /// Description for the radio button (selection controls) demo.
  ///
  /// In en, this message translates to:
  /// **'Radio buttons allow the user to select one option from a set. Use radio buttons for exclusive selection if you think that the user needs to see all available options side-by-side.'**
  String get demoSelectionControlsRadioDescription;

  /// Title for the switches (selection controls) demo.
  ///
  /// In en, this message translates to:
  /// **'Switch'**
  String get demoSelectionControlsSwitchTitle;

  /// Description for the switches (selection controls) demo.
  ///
  /// In en, this message translates to:
  /// **'On/off switches toggle the state of a single settings option. The option that the switch controls, as well as the state it\'s in, should be made clear from the corresponding inline label.'**
  String get demoSelectionControlsSwitchDescription;

  /// Title for text fields demo.
  ///
  /// In en, this message translates to:
  /// **'Text fields'**
  String get demoBottomTextFieldsTitle;

  /// Title for text fields demo.
  ///
  /// In en, this message translates to:
  /// **'Text fields'**
  String get demoTextFieldTitle;

  /// Description for text fields demo.
  ///
  /// In en, this message translates to:
  /// **'Single line of editable text and numbers'**
  String get demoTextFieldSubtitle;

  /// Description for text fields demo.
  ///
  /// In en, this message translates to:
  /// **'Text fields allow users to enter text into a UI. They typically appear in forms and dialogs.'**
  String get demoTextFieldDescription;

  /// Label for show password icon.
  ///
  /// In en, this message translates to:
  /// **'Show password'**
  String get demoTextFieldShowPasswordLabel;

  /// Label for hide password icon.
  ///
  /// In en, this message translates to:
  /// **'Hide password'**
  String get demoTextFieldHidePasswordLabel;

  /// Text that shows up on form errors.
  ///
  /// In en, this message translates to:
  /// **'Please fix the errors in red before submitting.'**
  String get demoTextFieldFormErrors;

  /// Shows up as submission error if name is not given in the form.
  ///
  /// In en, this message translates to:
  /// **'Name is required.'**
  String get demoTextFieldNameRequired;

  /// Error that shows if non-alphabetical characters are given.
  ///
  /// In en, this message translates to:
  /// **'Please enter only alphabetical characters.'**
  String get demoTextFieldOnlyAlphabeticalChars;

  /// Error that shows up if non-valid non-US phone number is given.
  ///
  /// In en, this message translates to:
  /// **'(###) ###-#### - Enter a US phone number.'**
  String get demoTextFieldEnterUSPhoneNumber;

  /// Error that shows up if password is not given.
  ///
  /// In en, this message translates to:
  /// **'Please enter a password.'**
  String get demoTextFieldEnterPassword;

  /// Error that shows up, if the re-typed password does not match the already given password.
  ///
  /// In en, this message translates to:
  /// **'The passwords don\'t match'**
  String get demoTextFieldPasswordsDoNotMatch;

  /// Placeholder for name field in form.
  ///
  /// In en, this message translates to:
  /// **'What do people call you?'**
  String get demoTextFieldWhatDoPeopleCallYou;

  /// The label for a name input field that is required (hence the star).
  ///
  /// In en, this message translates to:
  /// **'Name*'**
  String get demoTextFieldNameField;

  /// Placeholder for when entering a phone number in a form.
  ///
  /// In en, this message translates to:
  /// **'Where can we reach you?'**
  String get demoTextFieldWhereCanWeReachYou;

  /// The label for a phone number input field that is required (hence the star).
  ///
  /// In en, this message translates to:
  /// **'Phone number*'**
  String get demoTextFieldPhoneNumber;

  /// The label for an email address input field.
  ///
  /// In en, this message translates to:
  /// **'Your email address'**
  String get demoTextFieldYourEmailAddress;

  /// The label for an email address input field
  ///
  /// In en, this message translates to:
  /// **'Email'**
  String get demoTextFieldEmail;

  /// The placeholder text for biography/life story input field.
  ///
  /// In en, this message translates to:
  /// **'Tell us about yourself (e.g., write down what you do or what hobbies you have)'**
  String get demoTextFieldTellUsAboutYourself;

  /// Helper text for biography/life story input field.
  ///
  /// In en, this message translates to:
  /// **'Keep it short, this is just a demo.'**
  String get demoTextFieldKeepItShort;

  /// The label for biography/life story input field.
  ///
  /// In en, this message translates to:
  /// **'Life story'**
  String get demoTextFieldLifeStory;

  /// The label for salary input field.
  ///
  /// In en, this message translates to:
  /// **'Salary'**
  String get demoTextFieldSalary;

  /// US currency, used as suffix in input field for salary.
  ///
  /// In en, this message translates to:
  /// **'USD'**
  String get demoTextFieldUSD;

  /// Helper text for password input field.
  ///
  /// In en, this message translates to:
  /// **'No more than 8 characters.'**
  String get demoTextFieldNoMoreThan;

  /// Label for password input field, that is required (hence the star).
  ///
  /// In en, this message translates to:
  /// **'Password*'**
  String get demoTextFieldPassword;

  /// Label for repeat password input field.
  ///
  /// In en, this message translates to:
  /// **'Re-type password*'**
  String get demoTextFieldRetypePassword;

  /// The submit button text for form.
  ///
  /// In en, this message translates to:
  /// **'SUBMIT'**
  String get demoTextFieldSubmit;

  /// Text that shows up when valid phone number and name is submitted in form.
  ///
  /// In en, this message translates to:
  /// **'{name} phone number is {phoneNumber}'**
  String demoTextFieldNameHasPhoneNumber(Object name, Object phoneNumber);

  /// Helper text to indicate that * means that it is a required field.
  ///
  /// In en, this message translates to:
  /// **'* indicates required field'**
  String get demoTextFieldRequiredField;

  /// Title for tooltip demo.
  ///
  /// In en, this message translates to:
  /// **'Tooltips'**
  String get demoTooltipTitle;

  /// Subtitle for tooltip demo.
  ///
  /// In en, this message translates to:
  /// **'Short message displayed on long press or hover'**
  String get demoTooltipSubtitle;

  /// Description for tooltip demo.
  ///
  /// In en, this message translates to:
  /// **'Tooltips provide text labels that help explain the function of a button or other user interface action. Tooltips display informative text when users hover over, focus on, or long press an element.'**
  String get demoTooltipDescription;

  /// Instructions for how to trigger a tooltip in the tooltip demo.
  ///
  /// In en, this message translates to:
  /// **'Long press or hover to display the tooltip.'**
  String get demoTooltipInstructions;

  /// Title for Comments tab of bottom navigation.
  ///
  /// In en, this message translates to:
  /// **'Comments'**
  String get bottomNavigationCommentsTab;

  /// Title for Calendar tab of bottom navigation.
  ///
  /// In en, this message translates to:
  /// **'Calendar'**
  String get bottomNavigationCalendarTab;

  /// Title for Account tab of bottom navigation.
  ///
  /// In en, this message translates to:
  /// **'Account'**
  String get bottomNavigationAccountTab;

  /// Title for Alarm tab of bottom navigation.
  ///
  /// In en, this message translates to:
  /// **'Alarm'**
  String get bottomNavigationAlarmTab;

  /// Title for Camera tab of bottom navigation.
  ///
  /// In en, this message translates to:
  /// **'Camera'**
  String get bottomNavigationCameraTab;

  /// Accessibility label for the content placeholder in the bottom navigation demo
  ///
  /// In en, this message translates to:
  /// **'Placeholder for {title} tab'**
  String bottomNavigationContentPlaceholder(Object title);

  /// Tooltip text for a create button.
  ///
  /// In en, this message translates to:
  /// **'Create'**
  String get buttonTextCreate;

  /// Message displayed after an option is selected from a dialog
  ///
  /// In en, this message translates to:
  /// **'You selected: \"{value}\"'**
  String dialogSelectedOption(Object value);

  /// A chip component to turn on the lights.
  ///
  /// In en, this message translates to:
  /// **'Turn on lights'**
  String get chipTurnOnLights;

  /// A chip component to select a small size.
  ///
  /// In en, this message translates to:
  /// **'Small'**
  String get chipSmall;

  /// A chip component to select a medium size.
  ///
  /// In en, this message translates to:
  /// **'Medium'**
  String get chipMedium;

  /// A chip component to select a large size.
  ///
  /// In en, this message translates to:
  /// **'Large'**
  String get chipLarge;

  /// A chip component to filter selection by elevators.
  ///
  /// In en, this message translates to:
  /// **'Elevator'**
  String get chipElevator;

  /// A chip component to filter selection by washers.
  ///
  /// In en, this message translates to:
  /// **'Washer'**
  String get chipWasher;

  /// A chip component to filter selection by fireplaces.
  ///
  /// In en, this message translates to:
  /// **'Fireplace'**
  String get chipFireplace;

  /// A chip component to that indicates a biking selection.
  ///
  /// In en, this message translates to:
  /// **'Biking'**
  String get chipBiking;

  /// Used in the title of the demos.
  ///
  /// In en, this message translates to:
  /// **'Demo'**
  String get demo;

  /// Used as semantic label for a BottomAppBar.
  ///
  /// In en, this message translates to:
  /// **'Bottom app bar'**
  String get bottomAppBar;

  /// Indicates the loading process.
  ///
  /// In en, this message translates to:
  /// **'Loading'**
  String get loading;

  /// Alert dialog message to discard draft.
  ///
  /// In en, this message translates to:
  /// **'Discard draft?'**
  String get dialogDiscardTitle;

  /// Alert dialog title to use location services.
  ///
  /// In en, this message translates to:
  /// **'Use Google\'s location service?'**
  String get dialogLocationTitle;

  /// Alert dialog description to use location services.
  ///
  /// In en, this message translates to:
  /// **'Let Google help apps determine location. This means sending anonymous location data to Google, even when no apps are running.'**
  String get dialogLocationDescription;

  /// Alert dialog cancel option.
  ///
  /// In en, this message translates to:
  /// **'CANCEL'**
  String get dialogCancel;

  /// Alert dialog discard option.
  ///
  /// In en, this message translates to:
  /// **'DISCARD'**
  String get dialogDiscard;

  /// Alert dialog disagree option.
  ///
  /// In en, this message translates to:
  /// **'DISAGREE'**
  String get dialogDisagree;

  /// Alert dialog agree option.
  ///
  /// In en, this message translates to:
  /// **'AGREE'**
  String get dialogAgree;

  /// Alert dialog title for setting a backup account.
  ///
  /// In en, this message translates to:
  /// **'Set backup account'**
  String get dialogSetBackup;

  /// Alert dialog option for adding an account.
  ///
  /// In en, this message translates to:
  /// **'Add account'**
  String get dialogAddAccount;

  /// Button text to display a dialog.
  ///
  /// In en, this message translates to:
  /// **'SHOW DIALOG'**
  String get dialogShow;

  /// Title for full screen dialog demo.
  ///
  /// In en, this message translates to:
  /// **'Full Screen Dialog'**
  String get dialogFullscreenTitle;

  /// Save button for full screen dialog demo.
  ///
  /// In en, this message translates to:
  /// **'SAVE'**
  String get dialogFullscreenSave;

  /// Description for full screen dialog demo.
  ///
  /// In en, this message translates to:
  /// **'A full screen dialog demo'**
  String get dialogFullscreenDescription;

  /// Button text for a generic iOS-style button.
  ///
  /// In en, this message translates to:
  /// **'Button'**
  String get cupertinoButton;

  /// Button text for a iOS-style button with a filled background.
  ///
  /// In en, this message translates to:
  /// **'With Background'**
  String get cupertinoButtonWithBackground;

  /// iOS-style alert cancel option.
  ///
  /// In en, this message translates to:
  /// **'Cancel'**
  String get cupertinoAlertCancel;

  /// iOS-style alert discard option.
  ///
  /// In en, this message translates to:
  /// **'Discard'**
  String get cupertinoAlertDiscard;

  /// iOS-style alert title for location permission.
  ///
  /// In en, this message translates to:
  /// **'Allow \"Maps\" to access your location while you are using the app?'**
  String get cupertinoAlertLocationTitle;

  /// iOS-style alert description for location permission.
  ///
  /// In en, this message translates to:
  /// **'Your current location will be displayed on the map and used for directions, nearby search results, and estimated travel times.'**
  String get cupertinoAlertLocationDescription;

  /// iOS-style alert allow option.
  ///
  /// In en, this message translates to:
  /// **'Allow'**
  String get cupertinoAlertAllow;

  /// iOS-style alert don't allow option.
  ///
  /// In en, this message translates to:
  /// **'Don\'t Allow'**
  String get cupertinoAlertDontAllow;

  /// iOS-style alert title for selecting favorite dessert.
  ///
  /// In en, this message translates to:
  /// **'Select Favorite Dessert'**
  String get cupertinoAlertFavoriteDessert;

  /// iOS-style alert description for selecting favorite dessert.
  ///
  /// In en, this message translates to:
  /// **'Please select your favorite type of dessert from the list below. Your selection will be used to customize the suggested list of eateries in your area.'**
  String get cupertinoAlertDessertDescription;

  /// iOS-style alert cheesecake option.
  ///
  /// In en, this message translates to:
  /// **'Cheesecake'**
  String get cupertinoAlertCheesecake;

  /// iOS-style alert tiramisu option.
  ///
  /// In en, this message translates to:
  /// **'Tiramisu'**
  String get cupertinoAlertTiramisu;

  /// iOS-style alert apple pie option.
  ///
  /// In en, this message translates to:
  /// **'Apple Pie'**
  String get cupertinoAlertApplePie;

  /// iOS-style alert chocolate brownie option.
  ///
  /// In en, this message translates to:
  /// **'Chocolate Brownie'**
  String get cupertinoAlertChocolateBrownie;

  /// Button text to show iOS-style alert.
  ///
  /// In en, this message translates to:
  /// **'Show Alert'**
  String get cupertinoShowAlert;

  /// Tab title for the color red.
  ///
  /// In en, this message translates to:
  /// **'RED'**
  String get colorsRed;

  /// Tab title for the color pink.
  ///
  /// In en, this message translates to:
  /// **'PINK'**
  String get colorsPink;

  /// Tab title for the color purple.
  ///
  /// In en, this message translates to:
  /// **'PURPLE'**
  String get colorsPurple;

  /// Tab title for the color deep purple.
  ///
  /// In en, this message translates to:
  /// **'DEEP PURPLE'**
  String get colorsDeepPurple;

  /// Tab title for the color indigo.
  ///
  /// In en, this message translates to:
  /// **'INDIGO'**
  String get colorsIndigo;

  /// Tab title for the color blue.
  ///
  /// In en, this message translates to:
  /// **'BLUE'**
  String get colorsBlue;

  /// Tab title for the color light blue.
  ///
  /// In en, this message translates to:
  /// **'LIGHT BLUE'**
  String get colorsLightBlue;

  /// Tab title for the color cyan.
  ///
  /// In en, this message translates to:
  /// **'CYAN'**
  String get colorsCyan;

  /// Tab title for the color teal.
  ///
  /// In en, this message translates to:
  /// **'TEAL'**
  String get colorsTeal;

  /// Tab title for the color green.
  ///
  /// In en, this message translates to:
  /// **'GREEN'**
  String get colorsGreen;

  /// Tab title for the color light green.
  ///
  /// In en, this message translates to:
  /// **'LIGHT GREEN'**
  String get colorsLightGreen;

  /// Tab title for the color lime.
  ///
  /// In en, this message translates to:
  /// **'LIME'**
  String get colorsLime;

  /// Tab title for the color yellow.
  ///
  /// In en, this message translates to:
  /// **'YELLOW'**
  String get colorsYellow;

  /// Tab title for the color amber.
  ///
  /// In en, this message translates to:
  /// **'AMBER'**
  String get colorsAmber;

  /// Tab title for the color orange.
  ///
  /// In en, this message translates to:
  /// **'ORANGE'**
  String get colorsOrange;

  /// Tab title for the color deep orange.
  ///
  /// In en, this message translates to:
  /// **'DEEP ORANGE'**
  String get colorsDeepOrange;

  /// Tab title for the color brown.
  ///
  /// In en, this message translates to:
  /// **'BROWN'**
  String get colorsBrown;

  /// Tab title for the color grey.
  ///
  /// In en, this message translates to:
  /// **'GREY'**
  String get colorsGrey;

  /// Tab title for the color blue grey.
  ///
  /// In en, this message translates to:
  /// **'BLUE GREY'**
  String get colorsBlueGrey;

  /// Title for Chennai location.
  ///
  /// In en, this message translates to:
  /// **'Chennai'**
  String get placeChennai;

  /// Title for Tanjore location.
  ///
  /// In en, this message translates to:
  /// **'Tanjore'**
  String get placeTanjore;

  /// Title for Chettinad location.
  ///
  /// In en, this message translates to:
  /// **'Chettinad'**
  String get placeChettinad;

  /// Title for Pondicherry location.
  ///
  /// In en, this message translates to:
  /// **'Pondicherry'**
  String get placePondicherry;

  /// Title for Flower Market location.
  ///
  /// In en, this message translates to:
  /// **'Flower Market'**
  String get placeFlowerMarket;

  /// Title for Bronze Works location.
  ///
  /// In en, this message translates to:
  /// **'Bronze Works'**
  String get placeBronzeWorks;

  /// Title for Market location.
  ///
  /// In en, this message translates to:
  /// **'Market'**
  String get placeMarket;

  /// Title for Thanjavur Temple location.
  ///
  /// In en, this message translates to:
  /// **'Thanjavur Temple'**
  String get placeThanjavurTemple;

  /// Title for Salt Farm location.
  ///
  /// In en, this message translates to:
  /// **'Salt Farm'**
  String get placeSaltFarm;

  /// Title for image of people riding on scooters.
  ///
  /// In en, this message translates to:
  /// **'Scooters'**
  String get placeScooters;

  /// Title for an image of a silk maker.
  ///
  /// In en, this message translates to:
  /// **'Silk Maker'**
  String get placeSilkMaker;

  /// Title for an image of preparing lunch.
  ///
  /// In en, this message translates to:
  /// **'Lunch Prep'**
  String get placeLunchPrep;

  /// Title for Beach location.
  ///
  /// In en, this message translates to:
  /// **'Beach'**
  String get placeBeach;

  /// Title for an image of a fisherman.
  ///
  /// In en, this message translates to:
  /// **'Fisherman'**
  String get placeFisherman;

  /// The title and name for the starter app.
  ///
  /// In en, this message translates to:
  /// **'Starter app'**
  String get starterAppTitle;

  /// The description for the starter app.
  ///
  /// In en, this message translates to:
  /// **'A responsive starter layout'**
  String get starterAppDescription;

  /// Generic placeholder for button.
  ///
  /// In en, this message translates to:
  /// **'BUTTON'**
  String get starterAppGenericButton;

  /// Tooltip on add icon.
  ///
  /// In en, this message translates to:
  /// **'Add'**
  String get starterAppTooltipAdd;

  /// Tooltip on favorite icon.
  ///
  /// In en, this message translates to:
  /// **'Favorite'**
  String get starterAppTooltipFavorite;

  /// Tooltip on share icon.
  ///
  /// In en, this message translates to:
  /// **'Share'**
  String get starterAppTooltipShare;

  /// Tooltip on search icon.
  ///
  /// In en, this message translates to:
  /// **'Search'**
  String get starterAppTooltipSearch;

  /// Generic placeholder for title in app bar.
  ///
  /// In en, this message translates to:
  /// **'Title'**
  String get starterAppGenericTitle;

  /// Generic placeholder for subtitle in drawer.
  ///
  /// In en, this message translates to:
  /// **'Subtitle'**
  String get starterAppGenericSubtitle;

  /// Generic placeholder for headline in drawer.
  ///
  /// In en, this message translates to:
  /// **'Headline'**
  String get starterAppGenericHeadline;

  /// Generic placeholder for body text in drawer.
  ///
  /// In en, this message translates to:
  /// **'Body'**
  String get starterAppGenericBody;

  /// Generic placeholder drawer item.
  ///
  /// In en, this message translates to:
  /// **'Item {value}'**
  String starterAppDrawerItem(Object value);

  /// Caption for a menu page.
  ///
  /// In en, this message translates to:
  /// **'MENU'**
  String get shrineMenuCaption;

  /// A tab showing products from all categories.
  ///
  /// In en, this message translates to:
  /// **'ALL'**
  String get shrineCategoryNameAll;

  /// A category of products consisting of accessories (clothing items).
  ///
  /// In en, this message translates to:
  /// **'ACCESSORIES'**
  String get shrineCategoryNameAccessories;

  /// A category of products consisting of clothing.
  ///
  /// In en, this message translates to:
  /// **'CLOTHING'**
  String get shrineCategoryNameClothing;

  /// A category of products consisting of items used at home.
  ///
  /// In en, this message translates to:
  /// **'HOME'**
  String get shrineCategoryNameHome;

  /// Label for a logout button.
  ///
  /// In en, this message translates to:
  /// **'LOGOUT'**
  String get shrineLogoutButtonCaption;

  /// On the login screen, a label for a textfield for the user to input their username.
  ///
  /// In en, this message translates to:
  /// **'Username'**
  String get shrineLoginUsernameLabel;

  /// On the login screen, a label for a textfield for the user to input their password.
  ///
  /// In en, this message translates to:
  /// **'Password'**
  String get shrineLoginPasswordLabel;

  /// On the login screen, the caption for a button to cancel login.
  ///
  /// In en, this message translates to:
  /// **'CANCEL'**
  String get shrineCancelButtonCaption;

  /// On the login screen, the caption for a button to proceed login.
  ///
  /// In en, this message translates to:
  /// **'NEXT'**
  String get shrineNextButtonCaption;

  /// Caption for a shopping cart page.
  ///
  /// In en, this message translates to:
  /// **'CART'**
  String get shrineCartPageCaption;

  /// A text showing the number of items for a specific product.
  ///
  /// In en, this message translates to:
  /// **'Quantity: {quantity}'**
  String shrineProductQuantity(Object quantity);

  /// A text showing the unit price of each product. Used as: 'Quantity: 3 x $129'. The currency will be handled by the formatter.
  ///
  /// In en, this message translates to:
  /// **'x {price}'**
  String shrineProductPrice(Object price);

  /// A text showing the total number of items in the cart.
  ///
  /// In en, this message translates to:
  /// **'{quantity, plural, =0{NO ITEMS} =1{1 ITEM} other{{quantity} ITEMS}}'**
  String shrineCartItemCount(num quantity);

  /// Caption for a button used to clear the cart.
  ///
  /// In en, this message translates to:
  /// **'CLEAR CART'**
  String get shrineCartClearButtonCaption;

  /// Label for a text showing total price of the items in the cart.
  ///
  /// In en, this message translates to:
  /// **'TOTAL'**
  String get shrineCartTotalCaption;

  /// Label for a text showing the subtotal price of the items in the cart (excluding shipping and tax).
  ///
  /// In en, this message translates to:
  /// **'Subtotal:'**
  String get shrineCartSubtotalCaption;

  /// Label for a text showing the shipping cost for the items in the cart.
  ///
  /// In en, this message translates to:
  /// **'Shipping:'**
  String get shrineCartShippingCaption;

  /// Label for a text showing the tax for the items in the cart.
  ///
  /// In en, this message translates to:
  /// **'Tax:'**
  String get shrineCartTaxCaption;

  /// Name of the product 'Vagabond sack'.
  ///
  /// In en, this message translates to:
  /// **'Vagabond sack'**
  String get shrineProductVagabondSack;

  /// Name of the product 'Stella sunglasses'.
  ///
  /// In en, this message translates to:
  /// **'Stella sunglasses'**
  String get shrineProductStellaSunglasses;

  /// Name of the product 'Whitney belt'.
  ///
  /// In en, this message translates to:
  /// **'Whitney belt'**
  String get shrineProductWhitneyBelt;

  /// Name of the product 'Garden strand'.
  ///
  /// In en, this message translates to:
  /// **'Garden strand'**
  String get shrineProductGardenStrand;

  /// Name of the product 'Strut earrings'.
  ///
  /// In en, this message translates to:
  /// **'Strut earrings'**
  String get shrineProductStrutEarrings;

  /// Name of the product 'Varsity socks'.
  ///
  /// In en, this message translates to:
  /// **'Varsity socks'**
  String get shrineProductVarsitySocks;

  /// Name of the product 'Weave keyring'.
  ///
  /// In en, this message translates to:
  /// **'Weave keyring'**
  String get shrineProductWeaveKeyring;

  /// Name of the product 'Gatsby hat'.
  ///
  /// In en, this message translates to:
  /// **'Gatsby hat'**
  String get shrineProductGatsbyHat;

  /// Name of the product 'Shrug bag'.
  ///
  /// In en, this message translates to:
  /// **'Shrug bag'**
  String get shrineProductShrugBag;

  /// Name of the product 'Gilt desk trio'.
  ///
  /// In en, this message translates to:
  /// **'Gilt desk trio'**
  String get shrineProductGiltDeskTrio;

  /// Name of the product 'Copper wire rack'.
  ///
  /// In en, this message translates to:
  /// **'Copper wire rack'**
  String get shrineProductCopperWireRack;

  /// Name of the product 'Soothe ceramic set'.
  ///
  /// In en, this message translates to:
  /// **'Soothe ceramic set'**
  String get shrineProductSootheCeramicSet;

  /// Name of the product 'Hurrahs tea set'.
  ///
  /// In en, this message translates to:
  /// **'Hurrahs tea set'**
  String get shrineProductHurrahsTeaSet;

  /// Name of the product 'Blue stone mug'.
  ///
  /// In en, this message translates to:
  /// **'Blue stone mug'**
  String get shrineProductBlueStoneMug;

  /// Name of the product 'Rainwater tray'.
  ///
  /// In en, this message translates to:
  /// **'Rainwater tray'**
  String get shrineProductRainwaterTray;

  /// Name of the product 'Chambray napkins'.
  ///
  /// In en, this message translates to:
  /// **'Chambray napkins'**
  String get shrineProductChambrayNapkins;

  /// Name of the product 'Succulent planters'.
  ///
  /// In en, this message translates to:
  /// **'Succulent planters'**
  String get shrineProductSucculentPlanters;

  /// Name of the product 'Quartet table'.
  ///
  /// In en, this message translates to:
  /// **'Quartet table'**
  String get shrineProductQuartetTable;

  /// Name of the product 'Kitchen quattro'.
  ///
  /// In en, this message translates to:
  /// **'Kitchen quattro'**
  String get shrineProductKitchenQuattro;

  /// Name of the product 'Clay sweater'.
  ///
  /// In en, this message translates to:
  /// **'Clay sweater'**
  String get shrineProductClaySweater;

  /// Name of the product 'Sea tunic'.
  ///
  /// In en, this message translates to:
  /// **'Sea tunic'**
  String get shrineProductSeaTunic;

  /// Name of the product 'Plaster tunic'.
  ///
  /// In en, this message translates to:
  /// **'Plaster tunic'**
  String get shrineProductPlasterTunic;

  /// Name of the product 'White pinstripe shirt'.
  ///
  /// In en, this message translates to:
  /// **'White pinstripe shirt'**
  String get shrineProductWhitePinstripeShirt;

  /// Name of the product 'Chambray shirt'.
  ///
  /// In en, this message translates to:
  /// **'Chambray shirt'**
  String get shrineProductChambrayShirt;

  /// Name of the product 'Seabreeze sweater'.
  ///
  /// In en, this message translates to:
  /// **'Seabreeze sweater'**
  String get shrineProductSeabreezeSweater;

  /// Name of the product 'Gentry jacket'.
  ///
  /// In en, this message translates to:
  /// **'Gentry jacket'**
  String get shrineProductGentryJacket;

  /// Name of the product 'Navy trousers'.
  ///
  /// In en, this message translates to:
  /// **'Navy trousers'**
  String get shrineProductNavyTrousers;

  /// Name of the product 'Walter henley (white)'.
  ///
  /// In en, this message translates to:
  /// **'Walter henley (white)'**
  String get shrineProductWalterHenleyWhite;

  /// Name of the product 'Surf and perf shirt'.
  ///
  /// In en, this message translates to:
  /// **'Surf and perf shirt'**
  String get shrineProductSurfAndPerfShirt;

  /// Name of the product 'Ginger scarf'.
  ///
  /// In en, this message translates to:
  /// **'Ginger scarf'**
  String get shrineProductGingerScarf;

  /// Name of the product 'Ramona crossover'.
  ///
  /// In en, this message translates to:
  /// **'Ramona crossover'**
  String get shrineProductRamonaCrossover;

  /// Name of the product 'Classic white collar'.
  ///
  /// In en, this message translates to:
  /// **'Classic white collar'**
  String get shrineProductClassicWhiteCollar;

  /// Name of the product 'Cerise scallop tee'.
  ///
  /// In en, this message translates to:
  /// **'Cerise scallop tee'**
  String get shrineProductCeriseScallopTee;

  /// Name of the product 'Shoulder rolls tee'.
  ///
  /// In en, this message translates to:
  /// **'Shoulder rolls tee'**
  String get shrineProductShoulderRollsTee;

  /// Name of the product 'Grey slouch tank'.
  ///
  /// In en, this message translates to:
  /// **'Grey slouch tank'**
  String get shrineProductGreySlouchTank;

  /// Name of the product 'Sunshirt dress'.
  ///
  /// In en, this message translates to:
  /// **'Sunshirt dress'**
  String get shrineProductSunshirtDress;

  /// Name of the product 'Fine lines tee'.
  ///
  /// In en, this message translates to:
  /// **'Fine lines tee'**
  String get shrineProductFineLinesTee;

  /// The tooltip text for a search button. Also used as a semantic label, used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'Search'**
  String get shrineTooltipSearch;

  /// The tooltip text for a settings button. Also used as a semantic label, used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'Settings'**
  String get shrineTooltipSettings;

  /// The tooltip text for a menu button. Also used as a semantic label, used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'Open menu'**
  String get shrineTooltipOpenMenu;

  /// The tooltip text for a button to close a menu. Also used as a semantic label, used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'Close menu'**
  String get shrineTooltipCloseMenu;

  /// The tooltip text for a button to close the shopping cart page. Also used as a semantic label, used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'Close cart'**
  String get shrineTooltipCloseCart;

  /// The description of a shopping cart button containing some products. Used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'{quantity, plural, =0{Shopping cart, no items} =1{Shopping cart, 1 item} other{Shopping cart, {quantity} items}}'**
  String shrineScreenReaderCart(num quantity);

  /// An announcement made by screen readers, such as TalkBack and VoiceOver to indicate the action of a button for adding a product to the cart.
  ///
  /// In en, this message translates to:
  /// **'Add to cart'**
  String get shrineScreenReaderProductAddToCart;

  /// A tooltip for a button to remove a product. This will be read by screen readers, such as TalkBack and VoiceOver when a product is added to the shopping cart.
  ///
  /// In en, this message translates to:
  /// **'Remove {product}'**
  String shrineScreenReaderRemoveProductButton(Object product);

  /// The tooltip text for a button to remove an item (a product) in a shopping cart. Also used as a semantic label, used by screen readers, such as TalkBack and VoiceOver.
  ///
  /// In en, this message translates to:
  /// **'Remove item'**
  String get shrineTooltipRemoveItem;

  /// Form field label to enter the number of diners.
  ///
  /// In en, this message translates to:
  /// **'Diners'**
  String get craneFormDiners;

  /// Form field label to select a date.
  ///
  /// In en, this message translates to:
  /// **'Select Date'**
  String get craneFormDate;

  /// Form field label to select a time.
  ///
  /// In en, this message translates to:
  /// **'Select Time'**
  String get craneFormTime;

  /// Form field label to select a location.
  ///
  /// In en, this message translates to:
  /// **'Select Location'**
  String get craneFormLocation;

  /// Form field label to select the number of travellers.
  ///
  /// In en, this message translates to:
  /// **'Travelers'**
  String get craneFormTravelers;

  /// Form field label to choose a travel origin.
  ///
  /// In en, this message translates to:
  /// **'Choose Origin'**
  String get craneFormOrigin;

  /// Form field label to choose a travel destination.
  ///
  /// In en, this message translates to:
  /// **'Choose Destination'**
  String get craneFormDestination;

  /// Form field label to select multiple dates.
  ///
  /// In en, this message translates to:
  /// **'Select Dates'**
  String get craneFormDates;

  /// Generic text for an amount of hours, abbreviated to the shortest form. For example 1h. {hours} should remain untranslated.
  ///
  /// In en, this message translates to:
  /// **'{hours, plural, =1{1h} other{{hours}h}}'**
  String craneHours(num hours);

  /// Generic text for an amount of minutes, abbreviated to the shortest form. For example 15m. {minutes} should remain untranslated.
  ///
  /// In en, this message translates to:
  /// **'{minutes, plural, =1{1m} other{{minutes}m}}'**
  String craneMinutes(num minutes);

  /// A pattern to define the layout of a flight duration string. For example in English one might say 1h 15m. Translation should only rearrange the inputs. {hoursShortForm} would for example be replaced by 1h, already translated to the given locale. {minutesShortForm} would for example be replaced by 15m, already translated to the given locale.
  ///
  /// In en, this message translates to:
  /// **'{hoursShortForm} {minutesShortForm}'**
  String craneFlightDuration(Object hoursShortForm, Object minutesShortForm);

  /// Title for FLY tab.
  ///
  /// In en, this message translates to:
  /// **'FLY'**
  String get craneFly;

  /// Title for SLEEP tab.
  ///
  /// In en, this message translates to:
  /// **'SLEEP'**
  String get craneSleep;

  /// Title for EAT tab.
  ///
  /// In en, this message translates to:
  /// **'EAT'**
  String get craneEat;

  /// Subhead for FLY tab.
  ///
  /// In en, this message translates to:
  /// **'Explore Flights by Destination'**
  String get craneFlySubhead;

  /// Subhead for SLEEP tab.
  ///
  /// In en, this message translates to:
  /// **'Explore Properties by Destination'**
  String get craneSleepSubhead;

  /// Subhead for EAT tab.
  ///
  /// In en, this message translates to:
  /// **'Explore Restaurants by Destination'**
  String get craneEatSubhead;

  /// Label indicating if a flight is nonstop or how many layovers it includes.
  ///
  /// In en, this message translates to:
  /// **'{numberOfStops, plural, =0{Nonstop} =1{1 stop} other{{numberOfStops} stops}}'**
  String craneFlyStops(num numberOfStops);

  /// Text indicating the number of available properties (temporary rentals). Always plural.
  ///
  /// In en, this message translates to:
  /// **'{totalProperties, plural, =0{No Available Properties} =1{1 Available Properties} other{{totalProperties} Available Properties}}'**
  String craneSleepProperties(num totalProperties);

  /// Text indicating the number of restaurants. Always plural.
  ///
  /// In en, this message translates to:
  /// **'{totalRestaurants, plural, =0{No Restaurants} =1{1 Restaurant} other{{totalRestaurants} Restaurants}}'**
  String craneEatRestaurants(num totalRestaurants);

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Aspen, United States'**
  String get craneFly0;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Big Sur, United States'**
  String get craneFly1;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Khumbu Valley, Nepal'**
  String get craneFly2;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Machu Picchu, Peru'**
  String get craneFly3;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Malé, Maldives'**
  String get craneFly4;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Vitznau, Switzerland'**
  String get craneFly5;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Mexico City, Mexico'**
  String get craneFly6;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Mount Rushmore, United States'**
  String get craneFly7;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Singapore'**
  String get craneFly8;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Havana, Cuba'**
  String get craneFly9;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Cairo, Egypt'**
  String get craneFly10;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Lisbon, Portugal'**
  String get craneFly11;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Napa, United States'**
  String get craneFly12;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Bali, Indonesia'**
  String get craneFly13;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Malé, Maldives'**
  String get craneSleep0;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Aspen, United States'**
  String get craneSleep1;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Machu Picchu, Peru'**
  String get craneSleep2;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Havana, Cuba'**
  String get craneSleep3;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Vitznau, Switzerland'**
  String get craneSleep4;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Big Sur, United States'**
  String get craneSleep5;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Napa, United States'**
  String get craneSleep6;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Porto, Portugal'**
  String get craneSleep7;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Tulum, Mexico'**
  String get craneSleep8;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Lisbon, Portugal'**
  String get craneSleep9;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Cairo, Egypt'**
  String get craneSleep10;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Taipei, Taiwan'**
  String get craneSleep11;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Naples, Italy'**
  String get craneEat0;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Dallas, United States'**
  String get craneEat1;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Córdoba, Argentina'**
  String get craneEat2;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Portland, United States'**
  String get craneEat3;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Paris, France'**
  String get craneEat4;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Seoul, South Korea'**
  String get craneEat5;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Seattle, United States'**
  String get craneEat6;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Nashville, United States'**
  String get craneEat7;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Atlanta, United States'**
  String get craneEat8;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Madrid, Spain'**
  String get craneEat9;

  /// Label for city.
  ///
  /// In en, this message translates to:
  /// **'Lisbon, Portugal'**
  String get craneEat10;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Chalet in a snowy landscape with evergreen trees'**
  String get craneFly0SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Tent in a field'**
  String get craneFly1SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Prayer flags in front of snowy mountain'**
  String get craneFly2SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Machu Picchu citadel'**
  String get craneFly3SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Overwater bungalows'**
  String get craneFly4SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Lake-side hotel in front of mountains'**
  String get craneFly5SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Aerial view of Palacio de Bellas Artes'**
  String get craneFly6SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Mount Rushmore'**
  String get craneFly7SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Supertree Grove'**
  String get craneFly8SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Man leaning on an antique blue car'**
  String get craneFly9SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Al-Azhar Mosque towers during sunset'**
  String get craneFly10SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Brick lighthouse at sea'**
  String get craneFly11SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Pool with palm trees'**
  String get craneFly12SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Sea-side pool with palm trees'**
  String get craneFly13SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Overwater bungalows'**
  String get craneSleep0SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Chalet in a snowy landscape with evergreen trees'**
  String get craneSleep1SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Machu Picchu citadel'**
  String get craneSleep2SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Man leaning on an antique blue car'**
  String get craneSleep3SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Lake-side hotel in front of mountains'**
  String get craneSleep4SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Tent in a field'**
  String get craneSleep5SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Pool with palm trees'**
  String get craneSleep6SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Colorful apartments at Riberia Square'**
  String get craneSleep7SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Mayan ruins on a cliff above a beach'**
  String get craneSleep8SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Brick lighthouse at sea'**
  String get craneSleep9SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Al-Azhar Mosque towers during sunset'**
  String get craneSleep10SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Taipei 101 skyscraper'**
  String get craneSleep11SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Pizza in a wood-fired oven'**
  String get craneEat0SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Empty bar with diner-style stools'**
  String get craneEat1SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Burger'**
  String get craneEat2SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Korean taco'**
  String get craneEat3SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Chocolate dessert'**
  String get craneEat4SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Artsy restaurant seating area'**
  String get craneEat5SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Shrimp dish'**
  String get craneEat6SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Bakery entrance'**
  String get craneEat7SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Plate of crawfish'**
  String get craneEat8SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Cafe counter with pastries'**
  String get craneEat9SemanticLabel;

  /// Semantic label for an image.
  ///
  /// In en, this message translates to:
  /// **'Woman holding huge pastrami sandwich'**
  String get craneEat10SemanticLabel;

  /// Menu item for the front page of the news app.
  ///
  /// In en, this message translates to:
  /// **'Front Page'**
  String get fortnightlyMenuFrontPage;

  /// Menu item for the world news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'World'**
  String get fortnightlyMenuWorld;

  /// Menu item for the United States news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'US'**
  String get fortnightlyMenuUS;

  /// Menu item for the political news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Politics'**
  String get fortnightlyMenuPolitics;

  /// Menu item for the business news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Business'**
  String get fortnightlyMenuBusiness;

  /// Menu item for the tech news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Tech'**
  String get fortnightlyMenuTech;

  /// Menu item for the science news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Science'**
  String get fortnightlyMenuScience;

  /// Menu item for the sports news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Sports'**
  String get fortnightlyMenuSports;

  /// Menu item for the travel news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Travel'**
  String get fortnightlyMenuTravel;

  /// Menu item for the culture news section of the news app.
  ///
  /// In en, this message translates to:
  /// **'Culture'**
  String get fortnightlyMenuCulture;

  /// Hashtag for the tech design trending topic of the news app.
  ///
  /// In en, this message translates to:
  /// **'TechDesign'**
  String get fortnightlyTrendingTechDesign;

  /// Hashtag for the reform trending topic of the news app.
  ///
  /// In en, this message translates to:
  /// **'Reform'**
  String get fortnightlyTrendingReform;

  /// Hashtag for the healthcare revolution trending topic of the news app.
  ///
  /// In en, this message translates to:
  /// **'HealthcareRevolution'**
  String get fortnightlyTrendingHealthcareRevolution;

  /// Hashtag for the green army trending topic of the news app.
  ///
  /// In en, this message translates to:
  /// **'GreenArmy'**
  String get fortnightlyTrendingGreenArmy;

  /// Hashtag for the stocks trending topic of the news app.
  ///
  /// In en, this message translates to:
  /// **'Stocks'**
  String get fortnightlyTrendingStocks;

  /// Title for news section regarding the latest updates.
  ///
  /// In en, this message translates to:
  /// **'Latest Updates'**
  String get fortnightlyLatestUpdates;

  /// Headline for a news article about healthcare.
  ///
  /// In en, this message translates to:
  /// **'The Quiet, Yet Powerful Healthcare Revolution'**
  String get fortnightlyHeadlineHealthcare;

  /// Headline for a news article about war.
  ///
  /// In en, this message translates to:
  /// **'Divided American Lives During War'**
  String get fortnightlyHeadlineWar;

  /// Headline for a news article about gasoline.
  ///
  /// In en, this message translates to:
  /// **'The Future of Gasoline'**
  String get fortnightlyHeadlineGasoline;

  /// Headline for a news article about the green army.
  ///
  /// In en, this message translates to:
  /// **'Reforming The Green Army From Within'**
  String get fortnightlyHeadlineArmy;

  /// Headline for a news article about stocks.
  ///
  /// In en, this message translates to:
  /// **'As Stocks Stagnate, Many Look To Currency'**
  String get fortnightlyHeadlineStocks;

  /// Headline for a news article about fabric.
  ///
  /// In en, this message translates to:
  /// **'Designers Use Tech To Make Futuristic Fabrics'**
  String get fortnightlyHeadlineFabrics;

  /// Headline for a news article about feminists and partisanship.
  ///
  /// In en, this message translates to:
  /// **'Feminists Take On Partisanship'**
  String get fortnightlyHeadlineFeminists;

  /// Headline for a news article about bees.
  ///
  /// In en, this message translates to:
  /// **'Farmland Bees In Short Supply'**
  String get fortnightlyHeadlineBees;

  /// Text label for Inbox destination.
  ///
  /// In en, this message translates to:
  /// **'Inbox'**
  String get replyInboxLabel;

  /// Text label for Starred destination.
  ///
  /// In en, this message translates to:
  /// **'Starred'**
  String get replyStarredLabel;

  /// Text label for Sent destination.
  ///
  /// In en, this message translates to:
  /// **'Sent'**
  String get replySentLabel;

  /// Text label for Trash destination.
  ///
  /// In en, this message translates to:
  /// **'Trash'**
  String get replyTrashLabel;

  /// Text label for Spam destination.
  ///
  /// In en, this message translates to:
  /// **'Spam'**
  String get replySpamLabel;

  /// Text label for Drafts destination.
  ///
  /// In en, this message translates to:
  /// **'Drafts'**
  String get replyDraftsLabel;

  /// Option title for TwoPane demo on foldable devices.
  ///
  /// In en, this message translates to:
  /// **'Foldable'**
  String get demoTwoPaneFoldableLabel;

  /// Description for the foldable option configuration on the TwoPane demo.
  ///
  /// In en, this message translates to:
  /// **'This is how TwoPane behaves on a foldable device.'**
  String get demoTwoPaneFoldableDescription;

  /// Option title for TwoPane demo in small screen mode. Counterpart of the foldable option.
  ///
  /// In en, this message translates to:
  /// **'Small Screen'**
  String get demoTwoPaneSmallScreenLabel;

  /// Description for the small screen option configuration on the TwoPane demo.
  ///
  /// In en, this message translates to:
  /// **'This is how TwoPane behaves on a small screen device.'**
  String get demoTwoPaneSmallScreenDescription;

  /// Option title for TwoPane demo in tablet or desktop mode.
  ///
  /// In en, this message translates to:
  /// **'Tablet / Desktop'**
  String get demoTwoPaneTabletLabel;

  /// Description for the tablet / desktop option configuration on the TwoPane demo.
  ///
  /// In en, this message translates to:
  /// **'This is how TwoPane behaves on a larger screen like a tablet or desktop.'**
  String get demoTwoPaneTabletDescription;

  /// Title for the TwoPane widget demo.
  ///
  /// In en, this message translates to:
  /// **'TwoPane'**
  String get demoTwoPaneTitle;

  /// Subtitle for the TwoPane widget demo.
  ///
  /// In en, this message translates to:
  /// **'Responsive layouts on foldable, large, and small screens'**
  String get demoTwoPaneSubtitle;

  /// Tip for user, visible on the right side of the splash screen when Gallery runs on a foldable device.
  ///
  /// In en, this message translates to:
  /// **'Select a demo'**
  String get splashSelectDemo;

  /// Title of one of the panes in the TwoPane demo. It sits on top of a list of items.
  ///
  /// In en, this message translates to:
  /// **'List'**
  String get demoTwoPaneList;

  /// Title of one of the panes in the TwoPane demo, which shows details of the currently selected item.
  ///
  /// In en, this message translates to:
  /// **'Details'**
  String get demoTwoPaneDetails;

  /// Tip for user, visible on the right side of the TwoPane widget demo in the foldable configuration.
  ///
  /// In en, this message translates to:
  /// **'Select an item'**
  String get demoTwoPaneSelectItem;

  /// Generic item placeholder visible in the TwoPane widget demo.
  ///
  /// In en, this message translates to:
  /// **'Item {value}'**
  String demoTwoPaneItem(Object value);

  /// Generic item description or details visible in the TwoPane widget demo.
  ///
  /// In en, this message translates to:
  /// **'Item {value} details'**
  String demoTwoPaneItemDetails(Object value);
}

class _GalleryLocalizationsDelegate extends LocalizationsDelegate<GalleryLocalizations> {
  const _GalleryLocalizationsDelegate();

  @override
  Future<GalleryLocalizations> load(Locale locale) {
    return SynchronousFuture<GalleryLocalizations>(lookupGalleryLocalizations(locale));
  }

  @override
  bool isSupported(Locale locale) => <String>['en'].contains(locale.languageCode);

  @override
  bool shouldReload(_GalleryLocalizationsDelegate old) => false;
}

GalleryLocalizations lookupGalleryLocalizations(Locale locale) {
  // Lookup logic when language+country codes are specified.
  switch (locale.languageCode) {
    case 'en':
      {
        switch (locale.countryCode) {
          case 'IS':
            return GalleryLocalizationsEnIs();
        }
        break;
      }
  }

  // Lookup logic when only language code is specified.
  switch (locale.languageCode) {
    case 'en':
      return GalleryLocalizationsEn();
  }

  throw FlutterError(
    'GalleryLocalizations.delegate failed to load unsupported locale "$locale". This is likely '
    'an issue with the localizations generation tool. Please file an issue '
    'on GitHub with a reproducible sample app and the gen-l10n configuration '
    'that was used.',
  );
}
