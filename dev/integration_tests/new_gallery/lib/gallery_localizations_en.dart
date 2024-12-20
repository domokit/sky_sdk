// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:intl/intl.dart' as intl;

import 'gallery_localizations.dart';

/// The translations for English (`en`).
class GalleryLocalizationsEn extends GalleryLocalizations {
  GalleryLocalizationsEn([super.locale = 'en']);

  @override
  String githubRepo(Object repoName) {
    return '$repoName GitHub repository';
  }

  @override
  String aboutDialogDescription(Object repoLink) {
    return 'To see the source code for this app, please visit the $repoLink.';
  }

  @override
  String get deselect => 'Deselect';

  @override
  String get notSelected => 'Not selected';

  @override
  String get select => 'Select';

  @override
  String get selectable => 'Selectable (long press)';

  @override
  String get selected => 'Selected';

  @override
  String get signIn => 'SIGN IN';

  @override
  String get bannerDemoText =>
      'Your password was updated on your other device. Please sign in again.';

  @override
  String get bannerDemoResetText => 'Reset the banner';

  @override
  String get bannerDemoMultipleText => 'Multiple actions';

  @override
  String get bannerDemoLeadingText => 'Leading Icon';

  @override
  String get dismiss => 'DISMISS';

  @override
  String get backToGallery => 'Back to Gallery';

  @override
  String get cardsDemoExplore => 'Explore';

  @override
  String cardsDemoExploreSemantics(Object destinationName) {
    return 'Explore $destinationName';
  }

  @override
  String cardsDemoShareSemantics(Object destinationName) {
    return 'Share $destinationName';
  }

  @override
  String get cardsDemoTappable => 'Tappable';

  @override
  String get cardsDemoTravelDestinationTitle1 => 'Top 10 Cities to Visit in Tamil Nadu';

  @override
  String get cardsDemoTravelDestinationDescription1 => 'Number 10';

  @override
  String get cardsDemoTravelDestinationCity1 => 'Thanjavur';

  @override
  String get cardsDemoTravelDestinationLocation1 => 'Thanjavur, Tamil Nadu';

  @override
  String get cardsDemoTravelDestinationTitle2 => 'Artisans of Southern India';

  @override
  String get cardsDemoTravelDestinationDescription2 => 'Silk Spinners';

  @override
  String get cardsDemoTravelDestinationCity2 => 'Chettinad';

  @override
  String get cardsDemoTravelDestinationLocation2 => 'Sivaganga, Tamil Nadu';

  @override
  String get cardsDemoTravelDestinationTitle3 => 'Brihadisvara Temple';

  @override
  String get cardsDemoTravelDestinationDescription3 => 'Temples';

  @override
  String get homeHeaderGallery => 'Gallery';

  @override
  String get homeHeaderCategories => 'Categories';

  @override
  String get shrineDescription => 'A fashionable retail app';

  @override
  String get fortnightlyDescription => 'A content-focused news app';

  @override
  String get rallyDescription => 'A personal finance app';

  @override
  String get replyDescription => 'An efficient, focused email app';

  @override
  String get rallyAccountDataChecking => 'Checking';

  @override
  String get rallyAccountDataHomeSavings => 'Home Savings';

  @override
  String get rallyAccountDataCarSavings => 'Car Savings';

  @override
  String get rallyAccountDataVacation => 'Vacation';

  @override
  String get rallyAccountDetailDataAnnualPercentageYield => 'Annual Percentage Yield';

  @override
  String get rallyAccountDetailDataInterestRate => 'Interest Rate';

  @override
  String get rallyAccountDetailDataInterestYtd => 'Interest YTD';

  @override
  String get rallyAccountDetailDataInterestPaidLastYear => 'Interest Paid Last Year';

  @override
  String get rallyAccountDetailDataNextStatement => 'Next Statement';

  @override
  String get rallyAccountDetailDataAccountOwner => 'Account Owner';

  @override
  String get rallyBillDetailTotalAmount => 'Total Amount';

  @override
  String get rallyBillDetailAmountPaid => 'Amount Paid';

  @override
  String get rallyBillDetailAmountDue => 'Amount Due';

  @override
  String get rallyBudgetCategoryCoffeeShops => 'Coffee Shops';

  @override
  String get rallyBudgetCategoryGroceries => 'Groceries';

  @override
  String get rallyBudgetCategoryRestaurants => 'Restaurants';

  @override
  String get rallyBudgetCategoryClothing => 'Clothing';

  @override
  String get rallyBudgetDetailTotalCap => 'Total Cap';

  @override
  String get rallyBudgetDetailAmountUsed => 'Amount Used';

  @override
  String get rallyBudgetDetailAmountLeft => 'Amount Left';

  @override
  String get rallySettingsManageAccounts => 'Manage Accounts';

  @override
  String get rallySettingsTaxDocuments => 'Tax Documents';

  @override
  String get rallySettingsPasscodeAndTouchId => 'Passcode and Touch ID';

  @override
  String get rallySettingsNotifications => 'Notifications';

  @override
  String get rallySettingsPersonalInformation => 'Personal Information';

  @override
  String get rallySettingsPaperlessSettings => 'Paperless Settings';

  @override
  String get rallySettingsFindAtms => 'Find ATMs';

  @override
  String get rallySettingsHelp => 'Help';

  @override
  String get rallySettingsSignOut => 'Sign out';

  @override
  String get rallyAccountTotal => 'Total';

  @override
  String get rallyBillsDue => 'Due';

  @override
  String get rallyBudgetLeft => 'Left';

  @override
  String get rallyAccounts => 'Accounts';

  @override
  String get rallyBills => 'Bills';

  @override
  String get rallyBudgets => 'Budgets';

  @override
  String get rallyAlerts => 'Alerts';

  @override
  String get rallySeeAll => 'SEE ALL';

  @override
  String get rallyFinanceLeft => ' LEFT';

  @override
  String get rallyTitleOverview => 'OVERVIEW';

  @override
  String get rallyTitleAccounts => 'ACCOUNTS';

  @override
  String get rallyTitleBills => 'BILLS';

  @override
  String get rallyTitleBudgets => 'BUDGETS';

  @override
  String get rallyTitleSettings => 'SETTINGS';

  @override
  String get rallyLoginLoginToRally => 'Login to Rally';

  @override
  String get rallyLoginNoAccount => "Don't have an account?";

  @override
  String get rallyLoginSignUp => 'SIGN UP';

  @override
  String get rallyLoginUsername => 'Username';

  @override
  String get rallyLoginPassword => 'Password';

  @override
  String get rallyLoginLabelLogin => 'Login';

  @override
  String get rallyLoginRememberMe => 'Remember Me';

  @override
  String get rallyLoginButtonLogin => 'LOGIN';

  @override
  String rallyAlertsMessageHeadsUpShopping(Object percent) {
    return "Heads up, you've used up $percent of your Shopping budget for this month.";
  }

  @override
  String rallyAlertsMessageSpentOnRestaurants(Object amount) {
    return "You've spent $amount on Restaurants this week.";
  }

  @override
  String rallyAlertsMessageATMFees(Object amount) {
    return "You've spent $amount in ATM fees this month";
  }

  @override
  String rallyAlertsMessageCheckingAccount(Object percent) {
    return 'Good work! Your checking account is $percent higher than last month.';
  }

  @override
  String rallyAlertsMessageUnassignedTransactions(num count) {
    final String temp0 = intl.Intl.pluralLogic(
      count,
      locale: localeName,
      other:
          'Increase your potential tax deduction! Assign categories to $count unassigned transactions.',
      one: 'Increase your potential tax deduction! Assign categories to 1 unassigned transaction.',
    );
    return temp0;
  }

  @override
  String get rallySeeAllAccounts => 'See all accounts';

  @override
  String get rallySeeAllBills => 'See all bills';

  @override
  String get rallySeeAllBudgets => 'See all budgets';

  @override
  String rallyAccountAmount(Object accountName, Object accountNumber, Object amount) {
    return '$accountName account $accountNumber with $amount.';
  }

  @override
  String rallyBillAmount(Object billName, Object date, Object amount) {
    return '$billName bill due $date for $amount.';
  }

  @override
  String rallyBudgetAmount(
    Object budgetName,
    Object amountUsed,
    Object amountTotal,
    Object amountLeft,
  ) {
    return '$budgetName budget with $amountUsed used of $amountTotal, $amountLeft left';
  }

  @override
  String get craneDescription => 'A personalized travel app';

  @override
  String get homeCategoryReference => 'STYLES & OTHER';

  @override
  String get demoInvalidURL => "Couldn't display URL:";

  @override
  String get demoOptionsTooltip => 'Options';

  @override
  String get demoInfoTooltip => 'Info';

  @override
  String get demoCodeTooltip => 'Demo Code';

  @override
  String get demoDocumentationTooltip => 'API Documentation';

  @override
  String get demoFullscreenTooltip => 'Full Screen';

  @override
  String get demoCodeViewerCopyAll => 'COPY ALL';

  @override
  String get demoCodeViewerCopiedToClipboardMessage => 'Copied to clipboard.';

  @override
  String demoCodeViewerFailedToCopyToClipboardMessage(Object error) {
    return 'Failed to copy to clipboard: $error';
  }

  @override
  String get demoOptionsFeatureTitle => 'View options';

  @override
  String get demoOptionsFeatureDescription => 'Tap here to view available options for this demo.';

  @override
  String get settingsTitle => 'Settings';

  @override
  String get settingsButtonLabel => 'Settings';

  @override
  String get settingsButtonCloseLabel => 'Close settings';

  @override
  String get settingsSystemDefault => 'System';

  @override
  String get settingsTextScaling => 'Text scaling';

  @override
  String get settingsTextScalingSmall => 'Small';

  @override
  String get settingsTextScalingNormal => 'Normal';

  @override
  String get settingsTextScalingLarge => 'Large';

  @override
  String get settingsTextScalingHuge => 'Huge';

  @override
  String get settingsTextDirection => 'Text direction';

  @override
  String get settingsTextDirectionLocaleBased => 'Based on locale';

  @override
  String get settingsTextDirectionLTR => 'LTR';

  @override
  String get settingsTextDirectionRTL => 'RTL';

  @override
  String get settingsLocale => 'Locale';

  @override
  String get settingsPlatformMechanics => 'Platform mechanics';

  @override
  String get settingsTheme => 'Theme';

  @override
  String get settingsDarkTheme => 'Dark';

  @override
  String get settingsLightTheme => 'Light';

  @override
  String get settingsSlowMotion => 'Slow motion';

  @override
  String get settingsAbout => 'About Flutter Gallery';

  @override
  String get settingsFeedback => 'Send feedback';

  @override
  String get settingsAttribution => 'Designed by TOASTER in London';

  @override
  String get demoAppBarTitle => 'App bar';

  @override
  String get demoAppBarSubtitle =>
      'Displays information and actions relating to the current screen';

  @override
  String get demoAppBarDescription =>
      "The App bar provides content and actions related to the current screen. It's used for branding, screen titles, navigation, and actions";

  @override
  String get demoBottomAppBarTitle => 'Bottom app bar';

  @override
  String get demoBottomAppBarSubtitle => 'Displays navigation and actions at the bottom';

  @override
  String get demoBottomAppBarDescription =>
      'Bottom app bars provide access to a bottom navigation drawer and up to four actions, including the floating action button.';

  @override
  String get bottomAppBarNotch => 'Notch';

  @override
  String get bottomAppBarPosition => 'Floating Action Button Position';

  @override
  String get bottomAppBarPositionDockedEnd => 'Docked - End';

  @override
  String get bottomAppBarPositionDockedCenter => 'Docked - Center';

  @override
  String get bottomAppBarPositionFloatingEnd => 'Floating - End';

  @override
  String get bottomAppBarPositionFloatingCenter => 'Floating - Center';

  @override
  String get demoBannerTitle => 'Banner';

  @override
  String get demoBannerSubtitle => 'Displaying a banner within a list';

  @override
  String get demoBannerDescription =>
      'A banner displays an important, succinct message, and provides actions for users to address (or dismiss the banner). A user action is required for it to be dismissed.';

  @override
  String get demoBottomNavigationTitle => 'Bottom navigation';

  @override
  String get demoBottomNavigationSubtitle => 'Bottom navigation with cross-fading views';

  @override
  String get demoBottomNavigationPersistentLabels => 'Persistent labels';

  @override
  String get demoBottomNavigationSelectedLabel => 'Selected label';

  @override
  String get demoBottomNavigationDescription =>
      'Bottom navigation bars display three to five destinations at the bottom of a screen. Each destination is represented by an icon and an optional text label. When a bottom navigation icon is tapped, the user is taken to the top-level navigation destination associated with that icon.';

  @override
  String get demoButtonTitle => 'Buttons';

  @override
  String get demoButtonSubtitle => 'Text, elevated, outlined, and more';

  @override
  String get demoTextButtonTitle => 'Text Button';

  @override
  String get demoTextButtonDescription =>
      'A text button displays an ink splash on press but does not lift. Use text buttons on toolbars, in dialogs and inline with padding';

  @override
  String get demoElevatedButtonTitle => 'Elevated Button';

  @override
  String get demoElevatedButtonDescription =>
      'Elevated buttons add dimension to mostly flat layouts. They emphasize functions on busy or wide spaces.';

  @override
  String get demoOutlinedButtonTitle => 'Outlined Button';

  @override
  String get demoOutlinedButtonDescription =>
      'Outlined buttons become opaque and elevate when pressed. They are often paired with raised buttons to indicate an alternative, secondary action.';

  @override
  String get demoToggleButtonTitle => 'Toggle Buttons';

  @override
  String get demoToggleButtonDescription =>
      'Toggle buttons can be used to group related options. To emphasize groups of related toggle buttons, a group should share a common container';

  @override
  String get demoFloatingButtonTitle => 'Floating Action Button';

  @override
  String get demoFloatingButtonDescription =>
      'A floating action button is a circular icon button that hovers over content to promote a primary action in the application.';

  @override
  String get demoCardTitle => 'Cards';

  @override
  String get demoCardSubtitle => 'Baseline cards with rounded corners';

  @override
  String get demoChipTitle => 'Chips';

  @override
  String get demoCardDescription =>
      'A card is a sheet of Material used to represent some related information, for example an album, a geographical location, a meal, contact details, etc.';

  @override
  String get demoChipSubtitle => 'Compact elements that represent an input, attribute, or action';

  @override
  String get demoActionChipTitle => 'Action Chip';

  @override
  String get demoActionChipDescription =>
      'Action chips are a set of options which trigger an action related to primary content. Action chips should appear dynamically and contextually in a UI.';

  @override
  String get demoChoiceChipTitle => 'Choice Chip';

  @override
  String get demoChoiceChipDescription =>
      'Choice chips represent a single choice from a set. Choice chips contain related descriptive text or categories.';

  @override
  String get demoFilterChipTitle => 'Filter Chip';

  @override
  String get demoFilterChipDescription =>
      'Filter chips use tags or descriptive words as a way to filter content.';

  @override
  String get demoInputChipTitle => 'Input Chip';

  @override
  String get demoInputChipDescription =>
      'Input chips represent a complex piece of information, such as an entity (person, place, or thing) or conversational text, in a compact form.';

  @override
  String get demoDataTableTitle => 'Data Tables';

  @override
  String get demoDataTableSubtitle => 'Rows and columns of information';

  @override
  String get demoDataTableDescription =>
      "Data tables display information in a grid-like format of rows and columns. They organize information in a way that's easy to scan, so that users can look for patterns and insights.";

  @override
  String get dataTableHeader => 'Nutrition';

  @override
  String get dataTableColumnDessert => 'Dessert (1 serving)';

  @override
  String get dataTableColumnCalories => 'Calories';

  @override
  String get dataTableColumnFat => 'Fat (g)';

  @override
  String get dataTableColumnCarbs => 'Carbs (g)';

  @override
  String get dataTableColumnProtein => 'Protein (g)';

  @override
  String get dataTableColumnSodium => 'Sodium (mg)';

  @override
  String get dataTableColumnCalcium => 'Calcium (%)';

  @override
  String get dataTableColumnIron => 'Iron (%)';

  @override
  String get dataTableRowFrozenYogurt => 'Frozen yogurt';

  @override
  String get dataTableRowIceCreamSandwich => 'Ice cream sandwich';

  @override
  String get dataTableRowEclair => 'Eclair';

  @override
  String get dataTableRowCupcake => 'Cupcake';

  @override
  String get dataTableRowGingerbread => 'Gingerbread';

  @override
  String get dataTableRowJellyBean => 'Jelly bean';

  @override
  String get dataTableRowLollipop => 'Lollipop';

  @override
  String get dataTableRowHoneycomb => 'Honeycomb';

  @override
  String get dataTableRowDonut => 'Donut';

  @override
  String get dataTableRowApplePie => 'Apple pie';

  @override
  String dataTableRowWithSugar(Object value) {
    return '$value with sugar';
  }

  @override
  String dataTableRowWithHoney(Object value) {
    return '$value with honey';
  }

  @override
  String get demoDialogTitle => 'Dialogs';

  @override
  String get demoDialogSubtitle => 'Simple, alert, and fullscreen';

  @override
  String get demoAlertDialogTitle => 'Alert';

  @override
  String get demoAlertDialogDescription =>
      'An alert dialog informs the user about situations that require acknowledgement. An alert dialog has an optional title and an optional list of actions.';

  @override
  String get demoAlertTitleDialogTitle => 'Alert With Title';

  @override
  String get demoSimpleDialogTitle => 'Simple';

  @override
  String get demoSimpleDialogDescription =>
      'A simple dialog offers the user a choice between several options. A simple dialog has an optional title that is displayed above the choices.';

  @override
  String get demoDividerTitle => 'Divider';

  @override
  String get demoDividerSubtitle =>
      'A divider is a thin line that groups content in lists and layouts.';

  @override
  String get demoDividerDescription =>
      'Dividers can be used in lists, drawers, and elsewhere to separate content.';

  @override
  String get demoVerticalDividerTitle => 'Vertical Divider';

  @override
  String get demoGridListsTitle => 'Grid Lists';

  @override
  String get demoGridListsSubtitle => 'Row and column layout';

  @override
  String get demoGridListsDescription =>
      'Grid Lists are best suited for presenting homogeneous data, typically images. Each item in a grid list is called a tile.';

  @override
  String get demoGridListsImageOnlyTitle => 'Image only';

  @override
  String get demoGridListsHeaderTitle => 'With header';

  @override
  String get demoGridListsFooterTitle => 'With footer';

  @override
  String get demoSlidersTitle => 'Sliders';

  @override
  String get demoSlidersSubtitle => 'Widgets for selecting a value by swiping';

  @override
  String get demoSlidersDescription =>
      'Sliders reflect a range of values along a bar, from which users may select a single value. They are ideal for adjusting settings such as volume, brightness, or applying image filters.';

  @override
  String get demoRangeSlidersTitle => 'Range Sliders';

  @override
  String get demoRangeSlidersDescription =>
      'Sliders reflect a range of values along a bar. They can have icons on both ends of the bar that reflect a range of values. They are ideal for adjusting settings such as volume, brightness, or applying image filters.';

  @override
  String get demoCustomSlidersTitle => 'Custom Sliders';

  @override
  String get demoCustomSlidersDescription =>
      'Sliders reflect a range of values along a bar, from which users may select a single value or range of values. The sliders can be themed and customized.';

  @override
  String get demoSlidersContinuousWithEditableNumericalValue =>
      'Continuous with Editable Numerical Value';

  @override
  String get demoSlidersDiscrete => 'Discrete';

  @override
  String get demoSlidersDiscreteSliderWithCustomTheme => 'Discrete Slider with Custom Theme';

  @override
  String get demoSlidersContinuousRangeSliderWithCustomTheme =>
      'Continuous Range Slider with Custom Theme';

  @override
  String get demoSlidersContinuous => 'Continuous';

  @override
  String get demoSlidersEditableNumericalValue => 'Editable numerical value';

  @override
  String get demoMenuTitle => 'Menu';

  @override
  String get demoContextMenuTitle => 'Context menu';

  @override
  String get demoSectionedMenuTitle => 'Sectioned menu';

  @override
  String get demoSimpleMenuTitle => 'Simple menu';

  @override
  String get demoChecklistMenuTitle => 'Checklist menu';

  @override
  String get demoMenuSubtitle => 'Menu buttons and simple menus';

  @override
  String get demoMenuDescription =>
      'A menu displays a list of choices on a temporary surface. They appear when users interact with a button, action, or other control.';

  @override
  String get demoMenuItemValueOne => 'Menu item one';

  @override
  String get demoMenuItemValueTwo => 'Menu item two';

  @override
  String get demoMenuItemValueThree => 'Menu item three';

  @override
  String get demoMenuOne => 'One';

  @override
  String get demoMenuTwo => 'Two';

  @override
  String get demoMenuThree => 'Three';

  @override
  String get demoMenuFour => 'Four';

  @override
  String get demoMenuAnItemWithAContextMenuButton => 'An item with a context menu';

  @override
  String get demoMenuContextMenuItemOne => 'Context menu item one';

  @override
  String get demoMenuADisabledMenuItem => 'Disabled menu item';

  @override
  String get demoMenuContextMenuItemThree => 'Context menu item three';

  @override
  String get demoMenuAnItemWithASectionedMenu => 'An item with a sectioned menu';

  @override
  String get demoMenuPreview => 'Preview';

  @override
  String get demoMenuShare => 'Share';

  @override
  String get demoMenuGetLink => 'Get link';

  @override
  String get demoMenuRemove => 'Remove';

  @override
  String demoMenuSelected(Object value) {
    return 'Selected: $value';
  }

  @override
  String demoMenuChecked(Object value) {
    return 'Checked: $value';
  }

  @override
  String get demoNavigationDrawerTitle => 'Navigation Drawer';

  @override
  String get demoNavigationDrawerSubtitle => 'Displaying a drawer within appbar';

  @override
  String get demoNavigationDrawerDescription =>
      'A Material Design panel that slides in horizontally from the edge of the screen to show navigation links in an application.';

  @override
  String get demoNavigationDrawerUserName => 'User Name';

  @override
  String get demoNavigationDrawerUserEmail => 'user.name@example.com';

  @override
  String get demoNavigationDrawerToPageOne => 'Item One';

  @override
  String get demoNavigationDrawerToPageTwo => 'Item Two';

  @override
  String get demoNavigationDrawerText =>
      'Swipe from the edge or tap the upper-left icon to see the drawer';

  @override
  String get demoNavigationRailTitle => 'Navigation Rail';

  @override
  String get demoNavigationRailSubtitle => 'Displaying a Navigation Rail within an app';

  @override
  String get demoNavigationRailDescription =>
      'A material widget that is meant to be displayed at the left or right of an app to navigate between a small number of views, typically between three and five.';

  @override
  String get demoNavigationRailFirst => 'First';

  @override
  String get demoNavigationRailSecond => 'Second';

  @override
  String get demoNavigationRailThird => 'Third';

  @override
  String get demoMenuAnItemWithASimpleMenu => 'An item with a simple menu';

  @override
  String get demoMenuAnItemWithAChecklistMenu => 'An item with a checklist menu';

  @override
  String get demoFullscreenDialogTitle => 'Fullscreen';

  @override
  String get demoFullscreenDialogDescription =>
      'The fullscreenDialog property specifies whether the incoming page is a fullscreen modal dialog';

  @override
  String get demoCupertinoActivityIndicatorTitle => 'Activity indicator';

  @override
  String get demoCupertinoActivityIndicatorSubtitle => 'iOS-style activity indicators';

  @override
  String get demoCupertinoActivityIndicatorDescription =>
      'An iOS-style activity indicator that spins clockwise.';

  @override
  String get demoCupertinoButtonsTitle => 'Buttons';

  @override
  String get demoCupertinoButtonsSubtitle => 'iOS-style buttons';

  @override
  String get demoCupertinoButtonsDescription =>
      'An iOS-style button. It takes in text and/or an icon that fades out and in on touch. May optionally have a background.';

  @override
  String get demoCupertinoContextMenuTitle => 'Context Menu';

  @override
  String get demoCupertinoContextMenuSubtitle => 'iOS-style context menu';

  @override
  String get demoCupertinoContextMenuDescription =>
      'An iOS-style full screen contextual menu that appears when an element is long-pressed.';

  @override
  String get demoCupertinoContextMenuActionOne => 'Action one';

  @override
  String get demoCupertinoContextMenuActionTwo => 'Action two';

  @override
  String get demoCupertinoContextMenuActionText =>
      'Tap and hold the Flutter logo to see the context menu.';

  @override
  String get demoCupertinoAlertsTitle => 'Alerts';

  @override
  String get demoCupertinoAlertsSubtitle => 'iOS-style alert dialogs';

  @override
  String get demoCupertinoAlertTitle => 'Alert';

  @override
  String get demoCupertinoAlertDescription =>
      'An alert dialog informs the user about situations that require acknowledgement. An alert dialog has an optional title, optional content, and an optional list of actions. The title is displayed above the content and the actions are displayed below the content.';

  @override
  String get demoCupertinoAlertWithTitleTitle => 'Alert With Title';

  @override
  String get demoCupertinoAlertButtonsTitle => 'Alert With Buttons';

  @override
  String get demoCupertinoAlertButtonsOnlyTitle => 'Alert Buttons Only';

  @override
  String get demoCupertinoActionSheetTitle => 'Action Sheet';

  @override
  String get demoCupertinoActionSheetDescription =>
      'An action sheet is a specific style of alert that presents the user with a set of two or more choices related to the current context. An action sheet can have a title, an additional message, and a list of actions.';

  @override
  String get demoCupertinoNavigationBarTitle => 'Navigation bar';

  @override
  String get demoCupertinoNavigationBarSubtitle => 'iOS-style navigation bar';

  @override
  String get demoCupertinoNavigationBarDescription =>
      'An iOS-styled navigation bar. The navigation bar is a toolbar that minimally consists of a page title, in the middle of the toolbar.';

  @override
  String get demoCupertinoPickerTitle => 'Pickers';

  @override
  String get demoCupertinoPickerSubtitle => 'iOS-style pickers';

  @override
  String get demoCupertinoPickerDescription =>
      'An iOS-style picker widget that can be used to select strings, dates, times, or both date and time.';

  @override
  String get demoCupertinoPickerTimer => 'Timer';

  @override
  String get demoCupertinoPicker => 'Picker';

  @override
  String get demoCupertinoPickerDate => 'Date';

  @override
  String get demoCupertinoPickerTime => 'Time';

  @override
  String get demoCupertinoPickerDateTime => 'Date and Time';

  @override
  String get demoCupertinoPullToRefreshTitle => 'Pull to refresh';

  @override
  String get demoCupertinoPullToRefreshSubtitle => 'iOS-style pull to refresh control';

  @override
  String get demoCupertinoPullToRefreshDescription =>
      'A widget implementing the iOS-style pull to refresh content control.';

  @override
  String get demoCupertinoSegmentedControlTitle => 'Segmented control';

  @override
  String get demoCupertinoSegmentedControlSubtitle => 'iOS-style segmented control';

  @override
  String get demoCupertinoSegmentedControlDescription =>
      'Used to select between a number of mutually exclusive options. When one option in the segmented control is selected, the other options in the segmented control cease to be selected.';

  @override
  String get demoCupertinoSliderTitle => 'Slider';

  @override
  String get demoCupertinoSliderSubtitle => 'iOS-style slider';

  @override
  String get demoCupertinoSliderDescription =>
      'A slider can be used to select from either a continuous or a discrete set of values.';

  @override
  String demoCupertinoSliderContinuous(Object value) {
    return 'Continuous: $value';
  }

  @override
  String demoCupertinoSliderDiscrete(Object value) {
    return 'Discrete: $value';
  }

  @override
  String get demoCupertinoSwitchSubtitle => 'iOS-style switch';

  @override
  String get demoCupertinoSwitchDescription =>
      'A switch is used to toggle the on/off state of a single setting.';

  @override
  String get demoCupertinoTabBarTitle => 'Tab bar';

  @override
  String get demoCupertinoTabBarSubtitle => 'iOS-style bottom tab bar';

  @override
  String get demoCupertinoTabBarDescription =>
      'An iOS-style bottom navigation tab bar. Displays multiple tabs with one tab being active, the first tab by default.';

  @override
  String get cupertinoTabBarHomeTab => 'Home';

  @override
  String get cupertinoTabBarChatTab => 'Chat';

  @override
  String get cupertinoTabBarProfileTab => 'Profile';

  @override
  String get demoCupertinoTextFieldTitle => 'Text fields';

  @override
  String get demoCupertinoTextFieldSubtitle => 'iOS-style text fields';

  @override
  String get demoCupertinoTextFieldDescription =>
      'A text field lets the user enter text, either with a hardware keyboard or with an onscreen keyboard.';

  @override
  String get demoCupertinoTextFieldPIN => 'PIN';

  @override
  String get demoCupertinoSearchTextFieldTitle => 'Search text field';

  @override
  String get demoCupertinoSearchTextFieldSubtitle => 'iOS-style search text field';

  @override
  String get demoCupertinoSearchTextFieldDescription =>
      'A search text field that lets the user search by entering text, and that can offer and filter suggestions.';

  @override
  String get demoCupertinoSearchTextFieldPlaceholder => 'Enter some text';

  @override
  String get demoCupertinoScrollbarTitle => 'Scrollbar';

  @override
  String get demoCupertinoScrollbarSubtitle => 'iOS-style scrollbar';

  @override
  String get demoCupertinoScrollbarDescription => 'A scrollbar that wraps the given child';

  @override
  String get demoMotionTitle => 'Motion';

  @override
  String get demoMotionSubtitle => 'All of the predefined transition patterns';

  @override
  String get demoContainerTransformDemoInstructions => 'Cards, Lists & FAB';

  @override
  String get demoSharedXAxisDemoInstructions => 'Next and Back Buttons';

  @override
  String get demoSharedYAxisDemoInstructions => 'Sort by "Recently Played"';

  @override
  String get demoSharedZAxisDemoInstructions => 'Settings icon button';

  @override
  String get demoFadeThroughDemoInstructions => 'Bottom navigation';

  @override
  String get demoFadeScaleDemoInstructions => 'Modal and FAB';

  @override
  String get demoContainerTransformTitle => 'Container Transform';

  @override
  String get demoContainerTransformDescription =>
      'The container transform pattern is designed for transitions between UI elements that include a container. This pattern creates a visible connection between two UI elements';

  @override
  String get demoContainerTransformModalBottomSheetTitle => 'Fade mode';

  @override
  String get demoContainerTransformTypeFade => 'FADE';

  @override
  String get demoContainerTransformTypeFadeThrough => 'FADE THROUGH';

  @override
  String get demoMotionPlaceholderTitle => 'Title';

  @override
  String get demoMotionPlaceholderSubtitle => 'Secondary text';

  @override
  String get demoMotionSmallPlaceholderSubtitle => 'Secondary';

  @override
  String get demoMotionDetailsPageTitle => 'Details Page';

  @override
  String get demoMotionListTileTitle => 'List item';

  @override
  String get demoSharedAxisDescription =>
      'The shared axis pattern is used for transitions between the UI elements that have a spatial or navigational relationship. This pattern uses a shared transformation on the x, y, or z axis to reinforce the relationship between elements.';

  @override
  String get demoSharedXAxisTitle => 'Shared x-axis';

  @override
  String get demoSharedXAxisBackButtonText => 'BACK';

  @override
  String get demoSharedXAxisNextButtonText => 'NEXT';

  @override
  String get demoSharedXAxisCoursePageTitle => 'Streamline your courses';

  @override
  String get demoSharedXAxisCoursePageSubtitle =>
      'Bundled categories appear as groups in your feed. You can always change this later.';

  @override
  String get demoSharedXAxisArtsAndCraftsCourseTitle => 'Arts & Crafts';

  @override
  String get demoSharedXAxisBusinessCourseTitle => 'Business';

  @override
  String get demoSharedXAxisIllustrationCourseTitle => 'Illustration';

  @override
  String get demoSharedXAxisDesignCourseTitle => 'Design';

  @override
  String get demoSharedXAxisCulinaryCourseTitle => 'Culinary';

  @override
  String get demoSharedXAxisBundledCourseSubtitle => 'Bundled';

  @override
  String get demoSharedXAxisIndividualCourseSubtitle => 'Shown Individually';

  @override
  String get demoSharedXAxisSignInWelcomeText => 'Hi David Park';

  @override
  String get demoSharedXAxisSignInSubtitleText => 'Sign in with your account';

  @override
  String get demoSharedXAxisSignInTextFieldLabel => 'Email or phone number';

  @override
  String get demoSharedXAxisForgotEmailButtonText => 'FORGOT EMAIL?';

  @override
  String get demoSharedXAxisCreateAccountButtonText => 'CREATE ACCOUNT';

  @override
  String get demoSharedYAxisTitle => 'Shared y-axis';

  @override
  String get demoSharedYAxisAlbumCount => '268 albums';

  @override
  String get demoSharedYAxisAlphabeticalSortTitle => 'A-Z';

  @override
  String get demoSharedYAxisRecentSortTitle => 'Recently played';

  @override
  String get demoSharedYAxisAlbumTileTitle => 'Album';

  @override
  String get demoSharedYAxisAlbumTileSubtitle => 'Artist';

  @override
  String get demoSharedYAxisAlbumTileDurationUnit => 'min';

  @override
  String get demoSharedZAxisTitle => 'Shared z-axis';

  @override
  String get demoSharedZAxisSettingsPageTitle => 'Settings';

  @override
  String get demoSharedZAxisBurgerRecipeTitle => 'Burger';

  @override
  String get demoSharedZAxisBurgerRecipeDescription => 'Burger recipe';

  @override
  String get demoSharedZAxisSandwichRecipeTitle => 'Sandwich';

  @override
  String get demoSharedZAxisSandwichRecipeDescription => 'Sandwich recipe';

  @override
  String get demoSharedZAxisDessertRecipeTitle => 'Dessert';

  @override
  String get demoSharedZAxisDessertRecipeDescription => 'Dessert recipe';

  @override
  String get demoSharedZAxisShrimpPlateRecipeTitle => 'Shrimp';

  @override
  String get demoSharedZAxisShrimpPlateRecipeDescription => 'Shrimp plate recipe';

  @override
  String get demoSharedZAxisCrabPlateRecipeTitle => 'Crab';

  @override
  String get demoSharedZAxisCrabPlateRecipeDescription => 'Crab plate recipe';

  @override
  String get demoSharedZAxisBeefSandwichRecipeTitle => 'Beef Sandwich';

  @override
  String get demoSharedZAxisBeefSandwichRecipeDescription => 'Beef Sandwich recipe';

  @override
  String get demoSharedZAxisSavedRecipesListTitle => 'Saved Recipes';

  @override
  String get demoSharedZAxisProfileSettingLabel => 'Profile';

  @override
  String get demoSharedZAxisNotificationSettingLabel => 'Notifications';

  @override
  String get demoSharedZAxisPrivacySettingLabel => 'Privacy';

  @override
  String get demoSharedZAxisHelpSettingLabel => 'Help';

  @override
  String get demoFadeThroughTitle => 'Fade through';

  @override
  String get demoFadeThroughDescription =>
      'The fade through pattern is used for transitions between UI elements that do not have a strong relationship to each other.';

  @override
  String get demoFadeThroughAlbumsDestination => 'Albums';

  @override
  String get demoFadeThroughPhotosDestination => 'Photos';

  @override
  String get demoFadeThroughSearchDestination => 'Search';

  @override
  String get demoFadeThroughTextPlaceholder => '123 photos';

  @override
  String get demoFadeScaleTitle => 'Fade';

  @override
  String get demoFadeScaleDescription =>
      'The fade pattern is used for UI elements that enter or exit within the bounds of the screen, such as a dialog that fades in the center of the screen.';

  @override
  String get demoFadeScaleShowAlertDialogButton => 'SHOW MODAL';

  @override
  String get demoFadeScaleShowFabButton => 'SHOW FAB';

  @override
  String get demoFadeScaleHideFabButton => 'HIDE FAB';

  @override
  String get demoFadeScaleAlertDialogHeader => 'Alert Dialog';

  @override
  String get demoFadeScaleAlertDialogCancelButton => 'CANCEL';

  @override
  String get demoFadeScaleAlertDialogDiscardButton => 'DISCARD';

  @override
  String get demoColorsTitle => 'Colors';

  @override
  String get demoColorsSubtitle => 'All of the predefined colors';

  @override
  String get demoColorsDescription =>
      "Color and color swatch constants which represent Material Design's color palette.";

  @override
  String get demoTypographyTitle => 'Typography';

  @override
  String get demoTypographySubtitle => 'All of the predefined text styles';

  @override
  String get demoTypographyDescription =>
      'Definitions for the various typographical styles found in Material Design.';

  @override
  String get demo2dTransformationsTitle => '2D transformations';

  @override
  String get demo2dTransformationsSubtitle => 'Pan and zoom';

  @override
  String get demo2dTransformationsDescription =>
      'Tap to edit tiles, and use gestures to move around the scene. Drag to pan and pinch with two fingers to zoom. Press the reset button to return to the starting orientation.';

  @override
  String get demo2dTransformationsResetTooltip => 'Reset transformations';

  @override
  String get demo2dTransformationsEditTooltip => 'Edit tile';

  @override
  String get buttonText => 'BUTTON';

  @override
  String get demoBottomSheetTitle => 'Bottom sheet';

  @override
  String get demoBottomSheetSubtitle => 'Persistent and modal bottom sheets';

  @override
  String get demoBottomSheetPersistentTitle => 'Persistent bottom sheet';

  @override
  String get demoBottomSheetPersistentDescription =>
      'A persistent bottom sheet shows information that supplements the primary content of the app. A persistent bottom sheet remains visible even when the user interacts with other parts of the app.';

  @override
  String get demoBottomSheetModalTitle => 'Modal bottom sheet';

  @override
  String get demoBottomSheetModalDescription =>
      'A modal bottom sheet is an alternative to a menu or a dialog and prevents the user from interacting with the rest of the app.';

  @override
  String get demoBottomSheetAddLabel => 'Add';

  @override
  String get demoBottomSheetButtonText => 'SHOW BOTTOM SHEET';

  @override
  String get demoBottomSheetHeader => 'Header';

  @override
  String demoBottomSheetItem(Object value) {
    return 'Item $value';
  }

  @override
  String get demoListsTitle => 'Lists';

  @override
  String get demoListsSubtitle => 'Scrolling list layouts';

  @override
  String get demoListsDescription =>
      'A single fixed-height row that typically contains some text as well as a leading or trailing icon.';

  @override
  String get demoOneLineListsTitle => 'One Line';

  @override
  String get demoTwoLineListsTitle => 'Two Lines';

  @override
  String get demoListsSecondary => 'Secondary text';

  @override
  String get demoProgressIndicatorTitle => 'Progress indicators';

  @override
  String get demoProgressIndicatorSubtitle => 'Linear, circular, indeterminate';

  @override
  String get demoCircularProgressIndicatorTitle => 'Circular Progress Indicator';

  @override
  String get demoCircularProgressIndicatorDescription =>
      'A Material Design circular progress indicator, which spins to indicate that the application is busy.';

  @override
  String get demoLinearProgressIndicatorTitle => 'Linear Progress Indicator';

  @override
  String get demoLinearProgressIndicatorDescription =>
      'A Material Design linear progress indicator, also known as a progress bar.';

  @override
  String get demoPickersTitle => 'Pickers';

  @override
  String get demoPickersSubtitle => 'Date and time selection';

  @override
  String get demoDatePickerTitle => 'Date Picker';

  @override
  String get demoDatePickerDescription =>
      'Shows a dialog containing a Material Design date picker.';

  @override
  String get demoTimePickerTitle => 'Time Picker';

  @override
  String get demoTimePickerDescription =>
      'Shows a dialog containing a Material Design time picker.';

  @override
  String get demoDateRangePickerTitle => 'Date Range Picker';

  @override
  String get demoDateRangePickerDescription =>
      'Shows a dialog containing a Material Design date range picker.';

  @override
  String get demoPickersShowPicker => 'SHOW PICKER';

  @override
  String get demoTabsTitle => 'Tabs';

  @override
  String get demoTabsScrollingTitle => 'Scrolling';

  @override
  String get demoTabsNonScrollingTitle => 'Non-scrolling';

  @override
  String get demoTabsSubtitle => 'Tabs with independently scrollable views';

  @override
  String get demoTabsDescription =>
      'Tabs organize content across different screens, data sets, and other interactions.';

  @override
  String get demoSnackbarsTitle => 'Snackbars';

  @override
  String get demoSnackbarsSubtitle => 'Snackbars show messages at the bottom of the screen';

  @override
  String get demoSnackbarsDescription =>
      "Snackbars inform users of a process that an app has performed or will perform. They appear temporarily, towards the bottom of the screen. They shouldn't interrupt the user experience, and they don't require user input to disappear.";

  @override
  String get demoSnackbarsButtonLabel => 'SHOW A SNACKBAR';

  @override
  String get demoSnackbarsText => 'This is a snackbar.';

  @override
  String get demoSnackbarsActionButtonLabel => 'ACTION';

  @override
  String get demoSnackbarsAction => 'You pressed the snackbar action.';

  @override
  String get demoSelectionControlsTitle => 'Selection controls';

  @override
  String get demoSelectionControlsSubtitle => 'Checkboxes, radio buttons, and switches';

  @override
  String get demoSelectionControlsCheckboxTitle => 'Checkbox';

  @override
  String get demoSelectionControlsCheckboxDescription =>
      "Checkboxes allow the user to select multiple options from a set. A normal checkbox's value is true or false and a tristate checkbox's value can also be null.";

  @override
  String get demoSelectionControlsRadioTitle => 'Radio';

  @override
  String get demoSelectionControlsRadioDescription =>
      'Radio buttons allow the user to select one option from a set. Use radio buttons for exclusive selection if you think that the user needs to see all available options side-by-side.';

  @override
  String get demoSelectionControlsSwitchTitle => 'Switch';

  @override
  String get demoSelectionControlsSwitchDescription =>
      "On/off switches toggle the state of a single settings option. The option that the switch controls, as well as the state it's in, should be made clear from the corresponding inline label.";

  @override
  String get demoBottomTextFieldsTitle => 'Text fields';

  @override
  String get demoTextFieldTitle => 'Text fields';

  @override
  String get demoTextFieldSubtitle => 'Single line of editable text and numbers';

  @override
  String get demoTextFieldDescription =>
      'Text fields allow users to enter text into a UI. They typically appear in forms and dialogs.';

  @override
  String get demoTextFieldShowPasswordLabel => 'Show password';

  @override
  String get demoTextFieldHidePasswordLabel => 'Hide password';

  @override
  String get demoTextFieldFormErrors => 'Please fix the errors in red before submitting.';

  @override
  String get demoTextFieldNameRequired => 'Name is required.';

  @override
  String get demoTextFieldOnlyAlphabeticalChars => 'Please enter only alphabetical characters.';

  @override
  String get demoTextFieldEnterUSPhoneNumber => '(###) ###-#### - Enter a US phone number.';

  @override
  String get demoTextFieldEnterPassword => 'Please enter a password.';

  @override
  String get demoTextFieldPasswordsDoNotMatch => "The passwords don't match";

  @override
  String get demoTextFieldWhatDoPeopleCallYou => 'What do people call you?';

  @override
  String get demoTextFieldNameField => 'Name*';

  @override
  String get demoTextFieldWhereCanWeReachYou => 'Where can we reach you?';

  @override
  String get demoTextFieldPhoneNumber => 'Phone number*';

  @override
  String get demoTextFieldYourEmailAddress => 'Your email address';

  @override
  String get demoTextFieldEmail => 'Email';

  @override
  String get demoTextFieldTellUsAboutYourself =>
      'Tell us about yourself (e.g., write down what you do or what hobbies you have)';

  @override
  String get demoTextFieldKeepItShort => 'Keep it short, this is just a demo.';

  @override
  String get demoTextFieldLifeStory => 'Life story';

  @override
  String get demoTextFieldSalary => 'Salary';

  @override
  String get demoTextFieldUSD => 'USD';

  @override
  String get demoTextFieldNoMoreThan => 'No more than 8 characters.';

  @override
  String get demoTextFieldPassword => 'Password*';

  @override
  String get demoTextFieldRetypePassword => 'Re-type password*';

  @override
  String get demoTextFieldSubmit => 'SUBMIT';

  @override
  String demoTextFieldNameHasPhoneNumber(Object name, Object phoneNumber) {
    return '$name phone number is $phoneNumber';
  }

  @override
  String get demoTextFieldRequiredField => '* indicates required field';

  @override
  String get demoTooltipTitle => 'Tooltips';

  @override
  String get demoTooltipSubtitle => 'Short message displayed on long press or hover';

  @override
  String get demoTooltipDescription =>
      'Tooltips provide text labels that help explain the function of a button or other user interface action. Tooltips display informative text when users hover over, focus on, or long press an element.';

  @override
  String get demoTooltipInstructions => 'Long press or hover to display the tooltip.';

  @override
  String get bottomNavigationCommentsTab => 'Comments';

  @override
  String get bottomNavigationCalendarTab => 'Calendar';

  @override
  String get bottomNavigationAccountTab => 'Account';

  @override
  String get bottomNavigationAlarmTab => 'Alarm';

  @override
  String get bottomNavigationCameraTab => 'Camera';

  @override
  String bottomNavigationContentPlaceholder(Object title) {
    return 'Placeholder for $title tab';
  }

  @override
  String get buttonTextCreate => 'Create';

  @override
  String dialogSelectedOption(Object value) {
    return 'You selected: "$value"';
  }

  @override
  String get chipTurnOnLights => 'Turn on lights';

  @override
  String get chipSmall => 'Small';

  @override
  String get chipMedium => 'Medium';

  @override
  String get chipLarge => 'Large';

  @override
  String get chipElevator => 'Elevator';

  @override
  String get chipWasher => 'Washer';

  @override
  String get chipFireplace => 'Fireplace';

  @override
  String get chipBiking => 'Biking';

  @override
  String get demo => 'Demo';

  @override
  String get bottomAppBar => 'Bottom app bar';

  @override
  String get loading => 'Loading';

  @override
  String get dialogDiscardTitle => 'Discard draft?';

  @override
  String get dialogLocationTitle => "Use Google's location service?";

  @override
  String get dialogLocationDescription =>
      'Let Google help apps determine location. This means sending anonymous location data to Google, even when no apps are running.';

  @override
  String get dialogCancel => 'CANCEL';

  @override
  String get dialogDiscard => 'DISCARD';

  @override
  String get dialogDisagree => 'DISAGREE';

  @override
  String get dialogAgree => 'AGREE';

  @override
  String get dialogSetBackup => 'Set backup account';

  @override
  String get dialogAddAccount => 'Add account';

  @override
  String get dialogShow => 'SHOW DIALOG';

  @override
  String get dialogFullscreenTitle => 'Full Screen Dialog';

  @override
  String get dialogFullscreenSave => 'SAVE';

  @override
  String get dialogFullscreenDescription => 'A full screen dialog demo';

  @override
  String get cupertinoButton => 'Button';

  @override
  String get cupertinoButtonWithBackground => 'With Background';

  @override
  String get cupertinoAlertCancel => 'Cancel';

  @override
  String get cupertinoAlertDiscard => 'Discard';

  @override
  String get cupertinoAlertLocationTitle =>
      'Allow "Maps" to access your location while you are using the app?';

  @override
  String get cupertinoAlertLocationDescription =>
      'Your current location will be displayed on the map and used for directions, nearby search results, and estimated travel times.';

  @override
  String get cupertinoAlertAllow => 'Allow';

  @override
  String get cupertinoAlertDontAllow => "Don't Allow";

  @override
  String get cupertinoAlertFavoriteDessert => 'Select Favorite Dessert';

  @override
  String get cupertinoAlertDessertDescription =>
      'Please select your favorite type of dessert from the list below. Your selection will be used to customize the suggested list of eateries in your area.';

  @override
  String get cupertinoAlertCheesecake => 'Cheesecake';

  @override
  String get cupertinoAlertTiramisu => 'Tiramisu';

  @override
  String get cupertinoAlertApplePie => 'Apple Pie';

  @override
  String get cupertinoAlertChocolateBrownie => 'Chocolate Brownie';

  @override
  String get cupertinoShowAlert => 'Show Alert';

  @override
  String get colorsRed => 'RED';

  @override
  String get colorsPink => 'PINK';

  @override
  String get colorsPurple => 'PURPLE';

  @override
  String get colorsDeepPurple => 'DEEP PURPLE';

  @override
  String get colorsIndigo => 'INDIGO';

  @override
  String get colorsBlue => 'BLUE';

  @override
  String get colorsLightBlue => 'LIGHT BLUE';

  @override
  String get colorsCyan => 'CYAN';

  @override
  String get colorsTeal => 'TEAL';

  @override
  String get colorsGreen => 'GREEN';

  @override
  String get colorsLightGreen => 'LIGHT GREEN';

  @override
  String get colorsLime => 'LIME';

  @override
  String get colorsYellow => 'YELLOW';

  @override
  String get colorsAmber => 'AMBER';

  @override
  String get colorsOrange => 'ORANGE';

  @override
  String get colorsDeepOrange => 'DEEP ORANGE';

  @override
  String get colorsBrown => 'BROWN';

  @override
  String get colorsGrey => 'GREY';

  @override
  String get colorsBlueGrey => 'BLUE GREY';

  @override
  String get placeChennai => 'Chennai';

  @override
  String get placeTanjore => 'Tanjore';

  @override
  String get placeChettinad => 'Chettinad';

  @override
  String get placePondicherry => 'Pondicherry';

  @override
  String get placeFlowerMarket => 'Flower Market';

  @override
  String get placeBronzeWorks => 'Bronze Works';

  @override
  String get placeMarket => 'Market';

  @override
  String get placeThanjavurTemple => 'Thanjavur Temple';

  @override
  String get placeSaltFarm => 'Salt Farm';

  @override
  String get placeScooters => 'Scooters';

  @override
  String get placeSilkMaker => 'Silk Maker';

  @override
  String get placeLunchPrep => 'Lunch Prep';

  @override
  String get placeBeach => 'Beach';

  @override
  String get placeFisherman => 'Fisherman';

  @override
  String get starterAppTitle => 'Starter app';

  @override
  String get starterAppDescription => 'A responsive starter layout';

  @override
  String get starterAppGenericButton => 'BUTTON';

  @override
  String get starterAppTooltipAdd => 'Add';

  @override
  String get starterAppTooltipFavorite => 'Favorite';

  @override
  String get starterAppTooltipShare => 'Share';

  @override
  String get starterAppTooltipSearch => 'Search';

  @override
  String get starterAppGenericTitle => 'Title';

  @override
  String get starterAppGenericSubtitle => 'Subtitle';

  @override
  String get starterAppGenericHeadline => 'Headline';

  @override
  String get starterAppGenericBody => 'Body';

  @override
  String starterAppDrawerItem(Object value) {
    return 'Item $value';
  }

  @override
  String get shrineMenuCaption => 'MENU';

  @override
  String get shrineCategoryNameAll => 'ALL';

  @override
  String get shrineCategoryNameAccessories => 'ACCESSORIES';

  @override
  String get shrineCategoryNameClothing => 'CLOTHING';

  @override
  String get shrineCategoryNameHome => 'HOME';

  @override
  String get shrineLogoutButtonCaption => 'LOGOUT';

  @override
  String get shrineLoginUsernameLabel => 'Username';

  @override
  String get shrineLoginPasswordLabel => 'Password';

  @override
  String get shrineCancelButtonCaption => 'CANCEL';

  @override
  String get shrineNextButtonCaption => 'NEXT';

  @override
  String get shrineCartPageCaption => 'CART';

  @override
  String shrineProductQuantity(Object quantity) {
    return 'Quantity: $quantity';
  }

  @override
  String shrineProductPrice(Object price) {
    return 'x $price';
  }

  @override
  String shrineCartItemCount(num quantity) {
    return intl.Intl.pluralLogic(
      quantity,
      locale: localeName,
      other: '$quantity ITEMS',
      one: '1 ITEM',
      zero: 'NO ITEMS',
    );
  }

  @override
  String get shrineCartClearButtonCaption => 'CLEAR CART';

  @override
  String get shrineCartTotalCaption => 'TOTAL';

  @override
  String get shrineCartSubtotalCaption => 'Subtotal:';

  @override
  String get shrineCartShippingCaption => 'Shipping:';

  @override
  String get shrineCartTaxCaption => 'Tax:';

  @override
  String get shrineProductVagabondSack => 'Vagabond sack';

  @override
  String get shrineProductStellaSunglasses => 'Stella sunglasses';

  @override
  String get shrineProductWhitneyBelt => 'Whitney belt';

  @override
  String get shrineProductGardenStrand => 'Garden strand';

  @override
  String get shrineProductStrutEarrings => 'Strut earrings';

  @override
  String get shrineProductVarsitySocks => 'Varsity socks';

  @override
  String get shrineProductWeaveKeyring => 'Weave keyring';

  @override
  String get shrineProductGatsbyHat => 'Gatsby hat';

  @override
  String get shrineProductShrugBag => 'Shrug bag';

  @override
  String get shrineProductGiltDeskTrio => 'Gilt desk trio';

  @override
  String get shrineProductCopperWireRack => 'Copper wire rack';

  @override
  String get shrineProductSootheCeramicSet => 'Soothe ceramic set';

  @override
  String get shrineProductHurrahsTeaSet => 'Hurrahs tea set';

  @override
  String get shrineProductBlueStoneMug => 'Blue stone mug';

  @override
  String get shrineProductRainwaterTray => 'Rainwater tray';

  @override
  String get shrineProductChambrayNapkins => 'Chambray napkins';

  @override
  String get shrineProductSucculentPlanters => 'Succulent planters';

  @override
  String get shrineProductQuartetTable => 'Quartet table';

  @override
  String get shrineProductKitchenQuattro => 'Kitchen quattro';

  @override
  String get shrineProductClaySweater => 'Clay sweater';

  @override
  String get shrineProductSeaTunic => 'Sea tunic';

  @override
  String get shrineProductPlasterTunic => 'Plaster tunic';

  @override
  String get shrineProductWhitePinstripeShirt => 'White pinstripe shirt';

  @override
  String get shrineProductChambrayShirt => 'Chambray shirt';

  @override
  String get shrineProductSeabreezeSweater => 'Seabreeze sweater';

  @override
  String get shrineProductGentryJacket => 'Gentry jacket';

  @override
  String get shrineProductNavyTrousers => 'Navy trousers';

  @override
  String get shrineProductWalterHenleyWhite => 'Walter henley (white)';

  @override
  String get shrineProductSurfAndPerfShirt => 'Surf and perf shirt';

  @override
  String get shrineProductGingerScarf => 'Ginger scarf';

  @override
  String get shrineProductRamonaCrossover => 'Ramona crossover';

  @override
  String get shrineProductClassicWhiteCollar => 'Classic white collar';

  @override
  String get shrineProductCeriseScallopTee => 'Cerise scallop tee';

  @override
  String get shrineProductShoulderRollsTee => 'Shoulder rolls tee';

  @override
  String get shrineProductGreySlouchTank => 'Grey slouch tank';

  @override
  String get shrineProductSunshirtDress => 'Sunshirt dress';

  @override
  String get shrineProductFineLinesTee => 'Fine lines tee';

  @override
  String get shrineTooltipSearch => 'Search';

  @override
  String get shrineTooltipSettings => 'Settings';

  @override
  String get shrineTooltipOpenMenu => 'Open menu';

  @override
  String get shrineTooltipCloseMenu => 'Close menu';

  @override
  String get shrineTooltipCloseCart => 'Close cart';

  @override
  String shrineScreenReaderCart(num quantity) {
    return intl.Intl.pluralLogic(
      quantity,
      locale: localeName,
      other: 'Shopping cart, $quantity items',
      one: 'Shopping cart, 1 item',
      zero: 'Shopping cart, no items',
    );
  }

  @override
  String get shrineScreenReaderProductAddToCart => 'Add to cart';

  @override
  String shrineScreenReaderRemoveProductButton(Object product) {
    return 'Remove $product';
  }

  @override
  String get shrineTooltipRemoveItem => 'Remove item';

  @override
  String get craneFormDiners => 'Diners';

  @override
  String get craneFormDate => 'Select Date';

  @override
  String get craneFormTime => 'Select Time';

  @override
  String get craneFormLocation => 'Select Location';

  @override
  String get craneFormTravelers => 'Travelers';

  @override
  String get craneFormOrigin => 'Choose Origin';

  @override
  String get craneFormDestination => 'Choose Destination';

  @override
  String get craneFormDates => 'Select Dates';

  @override
  String craneHours(num hours) {
    final String temp0 = intl.Intl.pluralLogic(
      hours,
      locale: localeName,
      other: '${hours}h',
      one: '1h',
    );
    return temp0;
  }

  @override
  String craneMinutes(num minutes) {
    final String temp0 = intl.Intl.pluralLogic(
      minutes,
      locale: localeName,
      other: '${minutes}m',
      one: '1m',
    );
    return temp0;
  }

  @override
  String craneFlightDuration(Object hoursShortForm, Object minutesShortForm) {
    return '$hoursShortForm $minutesShortForm';
  }

  @override
  String get craneFly => 'FLY';

  @override
  String get craneSleep => 'SLEEP';

  @override
  String get craneEat => 'EAT';

  @override
  String get craneFlySubhead => 'Explore Flights by Destination';

  @override
  String get craneSleepSubhead => 'Explore Properties by Destination';

  @override
  String get craneEatSubhead => 'Explore Restaurants by Destination';

  @override
  String craneFlyStops(num numberOfStops) {
    final String temp0 = intl.Intl.pluralLogic(
      numberOfStops,
      locale: localeName,
      other: '$numberOfStops stops',
      one: '1 stop',
      zero: 'Nonstop',
    );
    return temp0;
  }

  @override
  String craneSleepProperties(num totalProperties) {
    final String temp0 = intl.Intl.pluralLogic(
      totalProperties,
      locale: localeName,
      other: '$totalProperties Available Properties',
      one: '1 Available Properties',
      zero: 'No Available Properties',
    );
    return temp0;
  }

  @override
  String craneEatRestaurants(num totalRestaurants) {
    final String temp0 = intl.Intl.pluralLogic(
      totalRestaurants,
      locale: localeName,
      other: '$totalRestaurants Restaurants',
      one: '1 Restaurant',
      zero: 'No Restaurants',
    );
    return temp0;
  }

  @override
  String get craneFly0 => 'Aspen, United States';

  @override
  String get craneFly1 => 'Big Sur, United States';

  @override
  String get craneFly2 => 'Khumbu Valley, Nepal';

  @override
  String get craneFly3 => 'Machu Picchu, Peru';

  @override
  String get craneFly4 => 'Malé, Maldives';

  @override
  String get craneFly5 => 'Vitznau, Switzerland';

  @override
  String get craneFly6 => 'Mexico City, Mexico';

  @override
  String get craneFly7 => 'Mount Rushmore, United States';

  @override
  String get craneFly8 => 'Singapore';

  @override
  String get craneFly9 => 'Havana, Cuba';

  @override
  String get craneFly10 => 'Cairo, Egypt';

  @override
  String get craneFly11 => 'Lisbon, Portugal';

  @override
  String get craneFly12 => 'Napa, United States';

  @override
  String get craneFly13 => 'Bali, Indonesia';

  @override
  String get craneSleep0 => 'Malé, Maldives';

  @override
  String get craneSleep1 => 'Aspen, United States';

  @override
  String get craneSleep2 => 'Machu Picchu, Peru';

  @override
  String get craneSleep3 => 'Havana, Cuba';

  @override
  String get craneSleep4 => 'Vitznau, Switzerland';

  @override
  String get craneSleep5 => 'Big Sur, United States';

  @override
  String get craneSleep6 => 'Napa, United States';

  @override
  String get craneSleep7 => 'Porto, Portugal';

  @override
  String get craneSleep8 => 'Tulum, Mexico';

  @override
  String get craneSleep9 => 'Lisbon, Portugal';

  @override
  String get craneSleep10 => 'Cairo, Egypt';

  @override
  String get craneSleep11 => 'Taipei, Taiwan';

  @override
  String get craneEat0 => 'Naples, Italy';

  @override
  String get craneEat1 => 'Dallas, United States';

  @override
  String get craneEat2 => 'Córdoba, Argentina';

  @override
  String get craneEat3 => 'Portland, United States';

  @override
  String get craneEat4 => 'Paris, France';

  @override
  String get craneEat5 => 'Seoul, South Korea';

  @override
  String get craneEat6 => 'Seattle, United States';

  @override
  String get craneEat7 => 'Nashville, United States';

  @override
  String get craneEat8 => 'Atlanta, United States';

  @override
  String get craneEat9 => 'Madrid, Spain';

  @override
  String get craneEat10 => 'Lisbon, Portugal';

  @override
  String get craneFly0SemanticLabel => 'Chalet in a snowy landscape with evergreen trees';

  @override
  String get craneFly1SemanticLabel => 'Tent in a field';

  @override
  String get craneFly2SemanticLabel => 'Prayer flags in front of snowy mountain';

  @override
  String get craneFly3SemanticLabel => 'Machu Picchu citadel';

  @override
  String get craneFly4SemanticLabel => 'Overwater bungalows';

  @override
  String get craneFly5SemanticLabel => 'Lake-side hotel in front of mountains';

  @override
  String get craneFly6SemanticLabel => 'Aerial view of Palacio de Bellas Artes';

  @override
  String get craneFly7SemanticLabel => 'Mount Rushmore';

  @override
  String get craneFly8SemanticLabel => 'Supertree Grove';

  @override
  String get craneFly9SemanticLabel => 'Man leaning on an antique blue car';

  @override
  String get craneFly10SemanticLabel => 'Al-Azhar Mosque towers during sunset';

  @override
  String get craneFly11SemanticLabel => 'Brick lighthouse at sea';

  @override
  String get craneFly12SemanticLabel => 'Pool with palm trees';

  @override
  String get craneFly13SemanticLabel => 'Sea-side pool with palm trees';

  @override
  String get craneSleep0SemanticLabel => 'Overwater bungalows';

  @override
  String get craneSleep1SemanticLabel => 'Chalet in a snowy landscape with evergreen trees';

  @override
  String get craneSleep2SemanticLabel => 'Machu Picchu citadel';

  @override
  String get craneSleep3SemanticLabel => 'Man leaning on an antique blue car';

  @override
  String get craneSleep4SemanticLabel => 'Lake-side hotel in front of mountains';

  @override
  String get craneSleep5SemanticLabel => 'Tent in a field';

  @override
  String get craneSleep6SemanticLabel => 'Pool with palm trees';

  @override
  String get craneSleep7SemanticLabel => 'Colorful apartments at Riberia Square';

  @override
  String get craneSleep8SemanticLabel => 'Mayan ruins on a cliff above a beach';

  @override
  String get craneSleep9SemanticLabel => 'Brick lighthouse at sea';

  @override
  String get craneSleep10SemanticLabel => 'Al-Azhar Mosque towers during sunset';

  @override
  String get craneSleep11SemanticLabel => 'Taipei 101 skyscraper';

  @override
  String get craneEat0SemanticLabel => 'Pizza in a wood-fired oven';

  @override
  String get craneEat1SemanticLabel => 'Empty bar with diner-style stools';

  @override
  String get craneEat2SemanticLabel => 'Burger';

  @override
  String get craneEat3SemanticLabel => 'Korean taco';

  @override
  String get craneEat4SemanticLabel => 'Chocolate dessert';

  @override
  String get craneEat5SemanticLabel => 'Artsy restaurant seating area';

  @override
  String get craneEat6SemanticLabel => 'Shrimp dish';

  @override
  String get craneEat7SemanticLabel => 'Bakery entrance';

  @override
  String get craneEat8SemanticLabel => 'Plate of crawfish';

  @override
  String get craneEat9SemanticLabel => 'Cafe counter with pastries';

  @override
  String get craneEat10SemanticLabel => 'Woman holding huge pastrami sandwich';

  @override
  String get fortnightlyMenuFrontPage => 'Front Page';

  @override
  String get fortnightlyMenuWorld => 'World';

  @override
  String get fortnightlyMenuUS => 'US';

  @override
  String get fortnightlyMenuPolitics => 'Politics';

  @override
  String get fortnightlyMenuBusiness => 'Business';

  @override
  String get fortnightlyMenuTech => 'Tech';

  @override
  String get fortnightlyMenuScience => 'Science';

  @override
  String get fortnightlyMenuSports => 'Sports';

  @override
  String get fortnightlyMenuTravel => 'Travel';

  @override
  String get fortnightlyMenuCulture => 'Culture';

  @override
  String get fortnightlyTrendingTechDesign => 'TechDesign';

  @override
  String get fortnightlyTrendingReform => 'Reform';

  @override
  String get fortnightlyTrendingHealthcareRevolution => 'HealthcareRevolution';

  @override
  String get fortnightlyTrendingGreenArmy => 'GreenArmy';

  @override
  String get fortnightlyTrendingStocks => 'Stocks';

  @override
  String get fortnightlyLatestUpdates => 'Latest Updates';

  @override
  String get fortnightlyHeadlineHealthcare => 'The Quiet, Yet Powerful Healthcare Revolution';

  @override
  String get fortnightlyHeadlineWar => 'Divided American Lives During War';

  @override
  String get fortnightlyHeadlineGasoline => 'The Future of Gasoline';

  @override
  String get fortnightlyHeadlineArmy => 'Reforming The Green Army From Within';

  @override
  String get fortnightlyHeadlineStocks => 'As Stocks Stagnate, Many Look To Currency';

  @override
  String get fortnightlyHeadlineFabrics => 'Designers Use Tech To Make Futuristic Fabrics';

  @override
  String get fortnightlyHeadlineFeminists => 'Feminists Take On Partisanship';

  @override
  String get fortnightlyHeadlineBees => 'Farmland Bees In Short Supply';

  @override
  String get replyInboxLabel => 'Inbox';

  @override
  String get replyStarredLabel => 'Starred';

  @override
  String get replySentLabel => 'Sent';

  @override
  String get replyTrashLabel => 'Trash';

  @override
  String get replySpamLabel => 'Spam';

  @override
  String get replyDraftsLabel => 'Drafts';

  @override
  String get demoTwoPaneFoldableLabel => 'Foldable';

  @override
  String get demoTwoPaneFoldableDescription => 'This is how TwoPane behaves on a foldable device.';

  @override
  String get demoTwoPaneSmallScreenLabel => 'Small Screen';

  @override
  String get demoTwoPaneSmallScreenDescription =>
      'This is how TwoPane behaves on a small screen device.';

  @override
  String get demoTwoPaneTabletLabel => 'Tablet / Desktop';

  @override
  String get demoTwoPaneTabletDescription =>
      'This is how TwoPane behaves on a larger screen like a tablet or desktop.';

  @override
  String get demoTwoPaneTitle => 'TwoPane';

  @override
  String get demoTwoPaneSubtitle => 'Responsive layouts on foldable, large, and small screens';

  @override
  String get splashSelectDemo => 'Select a demo';

  @override
  String get demoTwoPaneList => 'List';

  @override
  String get demoTwoPaneDetails => 'Details';

  @override
  String get demoTwoPaneSelectItem => 'Select an item';

  @override
  String demoTwoPaneItem(Object value) {
    return 'Item $value';
  }

  @override
  String demoTwoPaneItemDetails(Object value) {
    return 'Item $value details';
  }
}

/// The translations for English, as used in Iceland (`en_IS`).
class GalleryLocalizationsEnIs extends GalleryLocalizationsEn {
  GalleryLocalizationsEnIs() : super('en_IS');

  @override
  String githubRepo(Object repoName) {
    return '$repoName GitHub repository';
  }

  @override
  String aboutDialogDescription(Object repoLink) {
    return 'To see the source code for this app, please visit the $repoLink.';
  }

  @override
  String get deselect => 'Deselect';

  @override
  String get notSelected => 'Not selected';

  @override
  String get select => 'Select';

  @override
  String get selectable => 'Selectable (long press)';

  @override
  String get selected => 'Selected';

  @override
  String get signIn => 'SIGN IN';

  @override
  String get bannerDemoText =>
      'Your password was updated on your other device. Please sign in again.';

  @override
  String get bannerDemoResetText => 'Reset the banner';

  @override
  String get bannerDemoMultipleText => 'Multiple actions';

  @override
  String get bannerDemoLeadingText => 'Leading icon';

  @override
  String get dismiss => 'DISMISS';

  @override
  String get backToGallery => 'Back to Gallery';

  @override
  String get cardsDemoExplore => 'Explore';

  @override
  String cardsDemoExploreSemantics(Object destinationName) {
    return 'Explore $destinationName';
  }

  @override
  String cardsDemoShareSemantics(Object destinationName) {
    return 'Share $destinationName';
  }

  @override
  String get cardsDemoTappable => 'Tappable';

  @override
  String get cardsDemoTravelDestinationTitle1 => 'Top 10 cities to visit in Tamil Nadu';

  @override
  String get cardsDemoTravelDestinationDescription1 => 'Number 10';

  @override
  String get cardsDemoTravelDestinationCity1 => 'Thanjavur';

  @override
  String get cardsDemoTravelDestinationLocation1 => 'Thanjavur, Tamil Nadu';

  @override
  String get cardsDemoTravelDestinationTitle2 => 'Artisans of Southern India';

  @override
  String get cardsDemoTravelDestinationDescription2 => 'Silk spinners';

  @override
  String get cardsDemoTravelDestinationCity2 => 'Chettinad';

  @override
  String get cardsDemoTravelDestinationLocation2 => 'Sivaganga, Tamil Nadu';

  @override
  String get cardsDemoTravelDestinationTitle3 => 'Brihadisvara Temple';

  @override
  String get cardsDemoTravelDestinationDescription3 => 'Temples';

  @override
  String get homeHeaderGallery => 'Gallery';

  @override
  String get homeHeaderCategories => 'Categories';

  @override
  String get shrineDescription => 'A fashionable retail app';

  @override
  String get fortnightlyDescription => 'A content-focused news app';

  @override
  String get rallyDescription => 'A personal finance app';

  @override
  String get replyDescription => 'An efficient, focused email app';

  @override
  String get rallyAccountDataChecking => 'Current';

  @override
  String get rallyAccountDataHomeSavings => 'Home savings';

  @override
  String get rallyAccountDataCarSavings => 'Car savings';

  @override
  String get rallyAccountDataVacation => 'Holiday';

  @override
  String get rallyAccountDetailDataAnnualPercentageYield => 'Annual percentage yield';

  @override
  String get rallyAccountDetailDataInterestRate => 'Interest rate';

  @override
  String get rallyAccountDetailDataInterestYtd => 'Interest YTD';

  @override
  String get rallyAccountDetailDataInterestPaidLastYear => 'Interest paid last year';

  @override
  String get rallyAccountDetailDataNextStatement => 'Next statement';

  @override
  String get rallyAccountDetailDataAccountOwner => 'Account owner';

  @override
  String get rallyBillDetailTotalAmount => 'Total amount';

  @override
  String get rallyBillDetailAmountPaid => 'Amount paid';

  @override
  String get rallyBillDetailAmountDue => 'Amount due';

  @override
  String get rallyBudgetCategoryCoffeeShops => 'Coffee shops';

  @override
  String get rallyBudgetCategoryGroceries => 'Groceries';

  @override
  String get rallyBudgetCategoryRestaurants => 'Restaurants';

  @override
  String get rallyBudgetCategoryClothing => 'Clothing';

  @override
  String get rallyBudgetDetailTotalCap => 'Total cap';

  @override
  String get rallyBudgetDetailAmountUsed => 'Amount used';

  @override
  String get rallyBudgetDetailAmountLeft => 'Amount left';

  @override
  String get rallySettingsManageAccounts => 'Manage accounts';

  @override
  String get rallySettingsTaxDocuments => 'Tax documents';

  @override
  String get rallySettingsPasscodeAndTouchId => 'Passcode and Touch ID';

  @override
  String get rallySettingsNotifications => 'Notifications';

  @override
  String get rallySettingsPersonalInformation => 'Personal information';

  @override
  String get rallySettingsPaperlessSettings => 'Paperless settings';

  @override
  String get rallySettingsFindAtms => 'Find ATMs';

  @override
  String get rallySettingsHelp => 'Help';

  @override
  String get rallySettingsSignOut => 'Sign out';

  @override
  String get rallyAccountTotal => 'Total';

  @override
  String get rallyBillsDue => 'Due';

  @override
  String get rallyBudgetLeft => 'Left';

  @override
  String get rallyAccounts => 'Accounts';

  @override
  String get rallyBills => 'Bills';

  @override
  String get rallyBudgets => 'Budgets';

  @override
  String get rallyAlerts => 'Alerts';

  @override
  String get rallySeeAll => 'SEE ALL';

  @override
  String get rallyFinanceLeft => 'LEFT';

  @override
  String get rallyTitleOverview => 'OVERVIEW';

  @override
  String get rallyTitleAccounts => 'ACCOUNTS';

  @override
  String get rallyTitleBills => 'BILLS';

  @override
  String get rallyTitleBudgets => 'BUDGETS';

  @override
  String get rallyTitleSettings => 'SETTINGS';

  @override
  String get rallyLoginLoginToRally => 'Log in to Rally';

  @override
  String get rallyLoginNoAccount => "Don't have an account?";

  @override
  String get rallyLoginSignUp => 'SIGN UP';

  @override
  String get rallyLoginUsername => 'Username';

  @override
  String get rallyLoginPassword => 'Password';

  @override
  String get rallyLoginLabelLogin => 'Log in';

  @override
  String get rallyLoginRememberMe => 'Remember me';

  @override
  String get rallyLoginButtonLogin => 'LOGIN';

  @override
  String rallyAlertsMessageHeadsUpShopping(Object percent) {
    return "Heads up: you've used up $percent of your shopping budget for this month.";
  }

  @override
  String rallyAlertsMessageSpentOnRestaurants(Object amount) {
    return "You've spent $amount on restaurants this week.";
  }

  @override
  String rallyAlertsMessageATMFees(Object amount) {
    return "You've spent $amount in ATM fees this month";
  }

  @override
  String rallyAlertsMessageCheckingAccount(Object percent) {
    return 'Good work! Your current account is $percent higher than last month.';
  }

  @override
  String rallyAlertsMessageUnassignedTransactions(num count) {
    final String temp0 = intl.Intl.pluralLogic(
      count,
      locale: localeName,
      other:
          'Increase your potential tax deduction! Assign categories to $count unassigned transactions.',
      one: 'Increase your potential tax deduction! Assign categories to 1 unassigned transaction.',
    );
    return temp0;
  }

  @override
  String get rallySeeAllAccounts => 'See all accounts';

  @override
  String get rallySeeAllBills => 'See all bills';

  @override
  String get rallySeeAllBudgets => 'See all budgets';

  @override
  String rallyAccountAmount(Object accountName, Object accountNumber, Object amount) {
    return '$accountName account $accountNumber with $amount.';
  }

  @override
  String rallyBillAmount(Object billName, Object date, Object amount) {
    return '$billName bill due $date for $amount.';
  }

  @override
  String rallyBudgetAmount(
    Object budgetName,
    Object amountUsed,
    Object amountTotal,
    Object amountLeft,
  ) {
    return '$budgetName budget with $amountUsed used of $amountTotal, $amountLeft left';
  }

  @override
  String get craneDescription => 'A personalised travel app';

  @override
  String get homeCategoryReference => 'STYLES AND OTHER';

  @override
  String get demoInvalidURL => "Couldn't display URL:";

  @override
  String get demoOptionsTooltip => 'Options';

  @override
  String get demoInfoTooltip => 'Info';

  @override
  String get demoCodeTooltip => 'Demo code';

  @override
  String get demoDocumentationTooltip => 'API Documentation';

  @override
  String get demoFullscreenTooltip => 'Full screen';

  @override
  String get demoCodeViewerCopyAll => 'COPY ALL';

  @override
  String get demoCodeViewerCopiedToClipboardMessage => 'Copied to clipboard.';

  @override
  String demoCodeViewerFailedToCopyToClipboardMessage(Object error) {
    return 'Failed to copy to clipboard: $error';
  }

  @override
  String get demoOptionsFeatureTitle => 'View options';

  @override
  String get demoOptionsFeatureDescription => 'Tap here to view available options for this demo.';

  @override
  String get settingsTitle => 'Settings';

  @override
  String get settingsButtonLabel => 'Settings';

  @override
  String get settingsButtonCloseLabel => 'Close settings';

  @override
  String get settingsSystemDefault => 'System';

  @override
  String get settingsTextScaling => 'Text scaling';

  @override
  String get settingsTextScalingSmall => 'Small';

  @override
  String get settingsTextScalingNormal => 'Normal';

  @override
  String get settingsTextScalingLarge => 'Large';

  @override
  String get settingsTextScalingHuge => 'Huge';

  @override
  String get settingsTextDirection => 'Text direction';

  @override
  String get settingsTextDirectionLocaleBased => 'Based on locale';

  @override
  String get settingsTextDirectionLTR => 'LTR';

  @override
  String get settingsTextDirectionRTL => 'RTL';

  @override
  String get settingsLocale => 'Locale';

  @override
  String get settingsPlatformMechanics => 'Platform mechanics';

  @override
  String get settingsTheme => 'Theme';

  @override
  String get settingsDarkTheme => 'Dark';

  @override
  String get settingsLightTheme => 'Light';

  @override
  String get settingsSlowMotion => 'Slow motion';

  @override
  String get settingsAbout => 'About Flutter Gallery';

  @override
  String get settingsFeedback => 'Send feedback';

  @override
  String get settingsAttribution => 'Designed by TOASTER in London';

  @override
  String get demoAppBarTitle => 'App bar';

  @override
  String get demoAppBarSubtitle =>
      'Displays information and actions relating to the current screen';

  @override
  String get demoAppBarDescription =>
      "The app bar provides content and actions related to the current screen. It's used for branding, screen titles, navigation and actions";

  @override
  String get demoBottomAppBarTitle => 'Bottom app bar';

  @override
  String get demoBottomAppBarSubtitle => 'Displays navigation and actions at the bottom';

  @override
  String get demoBottomAppBarDescription =>
      'Bottom app bars provide access to a bottom navigation drawer and up to four actions, including the floating action button.';

  @override
  String get bottomAppBarNotch => 'Notch';

  @override
  String get bottomAppBarPosition => 'Floating action button position';

  @override
  String get bottomAppBarPositionDockedEnd => 'Docked - End';

  @override
  String get bottomAppBarPositionDockedCenter => 'Docked - Centre';

  @override
  String get bottomAppBarPositionFloatingEnd => 'Floating - End';

  @override
  String get bottomAppBarPositionFloatingCenter => 'Floating - Centre';

  @override
  String get demoBannerTitle => 'Banner';

  @override
  String get demoBannerSubtitle => 'Displaying a banner within a list';

  @override
  String get demoBannerDescription =>
      'A banner displays an important, succinct message, and provides actions for users to address (or dismiss the banner). A user action is required for it to be dismissed.';

  @override
  String get demoBottomNavigationTitle => 'Bottom navigation';

  @override
  String get demoBottomNavigationSubtitle => 'Bottom navigation with cross-fading views';

  @override
  String get demoBottomNavigationPersistentLabels => 'Persistent labels';

  @override
  String get demoBottomNavigationSelectedLabel => 'Selected label';

  @override
  String get demoBottomNavigationDescription =>
      'Bottom navigation bars display three to five destinations at the bottom of a screen. Each destination is represented by an icon and an optional text label. When a bottom navigation icon is tapped, the user is taken to the top-level navigation destination associated with that icon.';

  @override
  String get demoButtonTitle => 'Buttons';

  @override
  String get demoButtonSubtitle => 'Text, elevated, outlined and more';

  @override
  String get demoTextButtonTitle => 'Text button';

  @override
  String get demoTextButtonDescription =>
      'A text button displays an ink splash on press but does not lift. Use text buttons on toolbars, in dialogues and inline with padding';

  @override
  String get demoElevatedButtonTitle => 'Elevated button';

  @override
  String get demoElevatedButtonDescription =>
      'Elevated buttons add dimension to mostly flat layouts. They emphasise functions on busy or wide spaces.';

  @override
  String get demoOutlinedButtonTitle => 'Outlined button';

  @override
  String get demoOutlinedButtonDescription =>
      'Outlined buttons become opaque and elevate when pressed. They are often paired with raised buttons to indicate an alternative, secondary action.';

  @override
  String get demoToggleButtonTitle => 'Toggle Buttons';

  @override
  String get demoToggleButtonDescription =>
      'Toggle buttons can be used to group related options. To emphasise groups of related toggle buttons, a group should share a common container';

  @override
  String get demoFloatingButtonTitle => 'Floating Action Button';

  @override
  String get demoFloatingButtonDescription =>
      'A floating action button is a circular icon button that hovers over content to promote a primary action in the application.';

  @override
  String get demoCardTitle => 'Cards';

  @override
  String get demoCardSubtitle => 'Baseline cards with rounded corners';

  @override
  String get demoChipTitle => 'Chips';

  @override
  String get demoCardDescription =>
      'A card is a sheet of material used to represent some related information, for example, an album, a geographical location, a meal, contact details, etc.';

  @override
  String get demoChipSubtitle => 'Compact elements that represent an input, attribute or action';

  @override
  String get demoActionChipTitle => 'Action chip';

  @override
  String get demoActionChipDescription =>
      'Action chips are a set of options which trigger an action related to primary content. Action chips should appear dynamically and contextually in a UI.';

  @override
  String get demoChoiceChipTitle => 'Choice chip';

  @override
  String get demoChoiceChipDescription =>
      'Choice chips represent a single choice from a set. Choice chips contain related descriptive text or categories.';

  @override
  String get demoFilterChipTitle => 'Filter chip';

  @override
  String get demoFilterChipDescription =>
      'Filter chips use tags or descriptive words as a way to filter content.';

  @override
  String get demoInputChipTitle => 'Input chip';

  @override
  String get demoInputChipDescription =>
      'Input chips represent a complex piece of information, such as an entity (person, place or thing) or conversational text, in a compact form.';

  @override
  String get demoDataTableTitle => 'Data tables';

  @override
  String get demoDataTableSubtitle => 'Rows and columns of information';

  @override
  String get demoDataTableDescription =>
      "Data tables display information in a grid-like format of rows and columns. They organise information in a way that's easy to scan, so that users can look for patterns and insights.";

  @override
  String get dataTableHeader => 'Nutrition';

  @override
  String get dataTableColumnDessert => 'Dessert (1 serving)';

  @override
  String get dataTableColumnCalories => 'Calories';

  @override
  String get dataTableColumnFat => 'Fat (gm)';

  @override
  String get dataTableColumnCarbs => 'Carbs (gm)';

  @override
  String get dataTableColumnProtein => 'Protein (gm)';

  @override
  String get dataTableColumnSodium => 'Sodium (mg)';

  @override
  String get dataTableColumnCalcium => 'Calcium (%)';

  @override
  String get dataTableColumnIron => 'Iron (%)';

  @override
  String get dataTableRowFrozenYogurt => 'Frozen yogurt';

  @override
  String get dataTableRowIceCreamSandwich => 'Ice cream sandwich';

  @override
  String get dataTableRowEclair => 'Eclair';

  @override
  String get dataTableRowCupcake => 'Cupcake';

  @override
  String get dataTableRowGingerbread => 'Gingerbread';

  @override
  String get dataTableRowJellyBean => 'Jelly bean';

  @override
  String get dataTableRowLollipop => 'Lollipop';

  @override
  String get dataTableRowHoneycomb => 'Honeycomb';

  @override
  String get dataTableRowDonut => 'Doughnut';

  @override
  String get dataTableRowApplePie => 'Apple pie';

  @override
  String dataTableRowWithSugar(Object value) {
    return '$value with sugar';
  }

  @override
  String dataTableRowWithHoney(Object value) {
    return '$value with honey';
  }

  @override
  String get demoDialogTitle => 'Dialogues';

  @override
  String get demoDialogSubtitle => 'Simple, alert and full-screen';

  @override
  String get demoAlertDialogTitle => 'Alert';

  @override
  String get demoAlertDialogDescription =>
      'An alert dialogue informs the user about situations that require acknowledgement. An alert dialogue has an optional title and an optional list of actions.';

  @override
  String get demoAlertTitleDialogTitle => 'Alert With Title';

  @override
  String get demoSimpleDialogTitle => 'Simple';

  @override
  String get demoSimpleDialogDescription =>
      'A simple dialogue offers the user a choice between several options. A simple dialogue has an optional title that is displayed above the choices.';

  @override
  String get demoDividerTitle => 'Divider';

  @override
  String get demoDividerSubtitle =>
      'A divider is a thin line that groups content in lists and layouts.';

  @override
  String get demoDividerDescription =>
      'Dividers can be used in lists, drawers and elsewhere to separate content.';

  @override
  String get demoVerticalDividerTitle => 'Vertical divider';

  @override
  String get demoGridListsTitle => 'Grid lists';

  @override
  String get demoGridListsSubtitle => 'Row and column layout';

  @override
  String get demoGridListsDescription =>
      'Grid lists are best suited for presenting homogeneous data, typically images. Each item in a grid list is called a tile.';

  @override
  String get demoGridListsImageOnlyTitle => 'Image only';

  @override
  String get demoGridListsHeaderTitle => 'With header';

  @override
  String get demoGridListsFooterTitle => 'With footer';

  @override
  String get demoSlidersTitle => 'Sliders';

  @override
  String get demoSlidersSubtitle => 'Widgets for selecting a value by swiping';

  @override
  String get demoSlidersDescription =>
      'Sliders reflect a range of values along a bar, from which users may select a single value. They are ideal for adjusting settings such as volume, brightness or applying image filters.';

  @override
  String get demoRangeSlidersTitle => 'Range sliders';

  @override
  String get demoRangeSlidersDescription =>
      'Sliders reflect a range of values along a bar. They can have icons on both ends of the bar that reflect a range of values. They are ideal for adjusting settings such as volume, brightness or applying image filters.';

  @override
  String get demoCustomSlidersTitle => 'Custom sliders';

  @override
  String get demoCustomSlidersDescription =>
      'Sliders reflect a range of values along a bar, from which users may select a single value or range of values. The sliders can be themed and customised.';

  @override
  String get demoSlidersContinuousWithEditableNumericalValue =>
      'Continuous with editable numerical value';

  @override
  String get demoSlidersDiscrete => 'Discrete';

  @override
  String get demoSlidersDiscreteSliderWithCustomTheme => 'Discrete slider with custom theme';

  @override
  String get demoSlidersContinuousRangeSliderWithCustomTheme =>
      'Continuous range slider with custom theme';

  @override
  String get demoSlidersContinuous => 'Continuous';

  @override
  String get demoSlidersEditableNumericalValue => 'Editable numerical value';

  @override
  String get demoMenuTitle => 'Menu';

  @override
  String get demoContextMenuTitle => 'Context menu';

  @override
  String get demoSectionedMenuTitle => 'Sectioned menu';

  @override
  String get demoSimpleMenuTitle => 'Simple menu';

  @override
  String get demoChecklistMenuTitle => 'Checklist menu';

  @override
  String get demoMenuSubtitle => 'Menu buttons and simple menus';

  @override
  String get demoMenuDescription =>
      'A menu displays a list of choices on a temporary surface. They appear when users interact with a button, action or other control.';

  @override
  String get demoMenuItemValueOne => 'Menu item one';

  @override
  String get demoMenuItemValueTwo => 'Menu item two';

  @override
  String get demoMenuItemValueThree => 'Menu item three';

  @override
  String get demoMenuOne => 'One';

  @override
  String get demoMenuTwo => 'Two';

  @override
  String get demoMenuThree => 'Three';

  @override
  String get demoMenuFour => 'Four';

  @override
  String get demoMenuAnItemWithAContextMenuButton => 'An item with a context menu';

  @override
  String get demoMenuContextMenuItemOne => 'Context menu item one';

  @override
  String get demoMenuADisabledMenuItem => 'Disabled menu item';

  @override
  String get demoMenuContextMenuItemThree => 'Context menu item three';

  @override
  String get demoMenuAnItemWithASectionedMenu => 'An item with a sectioned menu';

  @override
  String get demoMenuPreview => 'Preview';

  @override
  String get demoMenuShare => 'Share';

  @override
  String get demoMenuGetLink => 'Get link';

  @override
  String get demoMenuRemove => 'Remove';

  @override
  String demoMenuSelected(Object value) {
    return 'Selected: $value';
  }

  @override
  String demoMenuChecked(Object value) {
    return 'Checked: $value';
  }

  @override
  String get demoNavigationDrawerTitle => 'Navigation drawer';

  @override
  String get demoNavigationDrawerSubtitle => 'Displaying a drawer within app bar';

  @override
  String get demoNavigationDrawerDescription =>
      'A Material Design panel that slides in horizontally from the edge of the screen to show navigation links in an application.';

  @override
  String get demoNavigationDrawerUserName => 'User name';

  @override
  String get demoNavigationDrawerUserEmail => 'user.name@example.com';

  @override
  String get demoNavigationDrawerToPageOne => 'Item one';

  @override
  String get demoNavigationDrawerToPageTwo => 'Item two';

  @override
  String get demoNavigationDrawerText =>
      'Swipe from the edge or tap the upper-left icon to see the drawer';

  @override
  String get demoNavigationRailTitle => 'Navigation rail';

  @override
  String get demoNavigationRailSubtitle => 'Displaying a navigation rail within an app';

  @override
  String get demoNavigationRailDescription =>
      'A material widget that is meant to be displayed at the left or right of an app to navigate between a small number of views, typically between three and five.';

  @override
  String get demoNavigationRailFirst => 'First';

  @override
  String get demoNavigationRailSecond => 'Second';

  @override
  String get demoNavigationRailThird => 'Third';

  @override
  String get demoMenuAnItemWithASimpleMenu => 'An item with a simple menu';

  @override
  String get demoMenuAnItemWithAChecklistMenu => 'An item with a checklist menu';

  @override
  String get demoFullscreenDialogTitle => 'Full screen';

  @override
  String get demoFullscreenDialogDescription =>
      'The fullscreenDialog property specifies whether the incoming page is a full-screen modal dialogue';

  @override
  String get demoCupertinoActivityIndicatorTitle => 'Activity indicator';

  @override
  String get demoCupertinoActivityIndicatorSubtitle => 'iOS-style activity indicators';

  @override
  String get demoCupertinoActivityIndicatorDescription =>
      'An iOS-style activity indicator that spins clockwise.';

  @override
  String get demoCupertinoButtonsTitle => 'Buttons';

  @override
  String get demoCupertinoButtonsSubtitle => 'iOS-style buttons';

  @override
  String get demoCupertinoButtonsDescription =>
      'An iOS-style button. It takes in text and/or an icon that fades out and in on touch. May optionally have a background.';

  @override
  String get demoCupertinoContextMenuTitle => 'Context menu';

  @override
  String get demoCupertinoContextMenuSubtitle => 'iOS-style context menu';

  @override
  String get demoCupertinoContextMenuDescription =>
      'An iOS-style full screen contextual menu that appears when an element is long-pressed.';

  @override
  String get demoCupertinoContextMenuActionOne => 'Action one';

  @override
  String get demoCupertinoContextMenuActionTwo => 'Action two';

  @override
  String get demoCupertinoContextMenuActionText =>
      'Tap and hold the Flutter logo to see the context menu.';

  @override
  String get demoCupertinoAlertsTitle => 'Alerts';

  @override
  String get demoCupertinoAlertsSubtitle => 'iOS-style alert dialogues';

  @override
  String get demoCupertinoAlertTitle => 'Alert';

  @override
  String get demoCupertinoAlertDescription =>
      'An alert dialogue informs the user about situations that require acknowledgement. An alert dialogue has an optional title, optional content and an optional list of actions. The title is displayed above the content and the actions are displayed below the content.';

  @override
  String get demoCupertinoAlertWithTitleTitle => 'Alert with title';

  @override
  String get demoCupertinoAlertButtonsTitle => 'Alert With Buttons';

  @override
  String get demoCupertinoAlertButtonsOnlyTitle => 'Alert Buttons Only';

  @override
  String get demoCupertinoActionSheetTitle => 'Action Sheet';

  @override
  String get demoCupertinoActionSheetDescription =>
      'An action sheet is a specific style of alert that presents the user with a set of two or more choices related to the current context. An action sheet can have a title, an additional message and a list of actions.';

  @override
  String get demoCupertinoNavigationBarTitle => 'Navigation bar';

  @override
  String get demoCupertinoNavigationBarSubtitle => 'iOS-style navigation bar';

  @override
  String get demoCupertinoNavigationBarDescription =>
      'An iOS-styled navigation bar. The navigation bar is a toolbar that minimally consists of a page title, in the middle of the toolbar.';

  @override
  String get demoCupertinoPickerTitle => 'Pickers';

  @override
  String get demoCupertinoPickerSubtitle => 'iOS-style pickers';

  @override
  String get demoCupertinoPickerDescription =>
      'An iOS-style picker widget that can be used to select strings, dates, times or both date and time.';

  @override
  String get demoCupertinoPickerTimer => 'Timer';

  @override
  String get demoCupertinoPicker => 'Picker';

  @override
  String get demoCupertinoPickerDate => 'Date';

  @override
  String get demoCupertinoPickerTime => 'Time';

  @override
  String get demoCupertinoPickerDateTime => 'Date and time';

  @override
  String get demoCupertinoPullToRefreshTitle => 'Pull to refresh';

  @override
  String get demoCupertinoPullToRefreshSubtitle => 'iOS-style pull to refresh control';

  @override
  String get demoCupertinoPullToRefreshDescription =>
      'A widget implementing the iOS-style pull to refresh content control.';

  @override
  String get demoCupertinoSegmentedControlTitle => 'Segmented control';

  @override
  String get demoCupertinoSegmentedControlSubtitle => 'iOS-style segmented control';

  @override
  String get demoCupertinoSegmentedControlDescription =>
      'Used to select between a number of mutually exclusive options. When one option in the segmented control is selected, the other options in the segmented control cease to be selected.';

  @override
  String get demoCupertinoSliderTitle => 'Slider';

  @override
  String get demoCupertinoSliderSubtitle => 'iOS-style slider';

  @override
  String get demoCupertinoSliderDescription =>
      'A slider can be used to select from either a continuous or a discrete set of values.';

  @override
  String demoCupertinoSliderContinuous(Object value) {
    return 'Continuous: $value';
  }

  @override
  String demoCupertinoSliderDiscrete(Object value) {
    return 'Discrete: $value';
  }

  @override
  String get demoCupertinoSwitchSubtitle => 'iOS-style switch';

  @override
  String get demoCupertinoSwitchDescription =>
      'A switch is used to toggle the on/off state of a single setting.';

  @override
  String get demoCupertinoTabBarTitle => 'Tab bar';

  @override
  String get demoCupertinoTabBarSubtitle => 'iOS-style bottom tab bar';

  @override
  String get demoCupertinoTabBarDescription =>
      'An iOS-style bottom navigation tab bar. Displays multiple tabs with one tab being active, the first tab by default.';

  @override
  String get cupertinoTabBarHomeTab => 'Home';

  @override
  String get cupertinoTabBarChatTab => 'Chat';

  @override
  String get cupertinoTabBarProfileTab => 'Profile';

  @override
  String get demoCupertinoTextFieldTitle => 'Text fields';

  @override
  String get demoCupertinoTextFieldSubtitle => 'iOS-style text fields';

  @override
  String get demoCupertinoTextFieldDescription =>
      'A text field allows the user to enter text, either with a hardware keyboard or with an on-screen keyboard.';

  @override
  String get demoCupertinoTextFieldPIN => 'PIN';

  @override
  String get demoCupertinoSearchTextFieldTitle => 'Search text field';

  @override
  String get demoCupertinoSearchTextFieldSubtitle => 'iOS-style search text field';

  @override
  String get demoCupertinoSearchTextFieldDescription =>
      'A search text field that lets the user search by entering text and that can offer and filter suggestions.';

  @override
  String get demoCupertinoSearchTextFieldPlaceholder => 'Enter some text';

  @override
  String get demoCupertinoScrollbarTitle => 'Scrollbar';

  @override
  String get demoCupertinoScrollbarSubtitle => 'iOS-style scrollbar';

  @override
  String get demoCupertinoScrollbarDescription => 'A scrollbar that wraps the given child';

  @override
  String get demoMotionTitle => 'Motion';

  @override
  String get demoMotionSubtitle => 'All of the predefined transition patterns';

  @override
  String get demoContainerTransformDemoInstructions => 'Cards, lists and FAB';

  @override
  String get demoSharedXAxisDemoInstructions => 'Next and back buttons';

  @override
  String get demoSharedYAxisDemoInstructions => "Sort by 'Recently played'";

  @override
  String get demoSharedZAxisDemoInstructions => 'Settings icon button';

  @override
  String get demoFadeThroughDemoInstructions => 'Bottom navigation';

  @override
  String get demoFadeScaleDemoInstructions => 'Modal and FAB';

  @override
  String get demoContainerTransformTitle => 'Container transform';

  @override
  String get demoContainerTransformDescription =>
      'The container transform pattern is designed for transitions between UI elements that include a container. This pattern creates a visible connection between two UI elements';

  @override
  String get demoContainerTransformModalBottomSheetTitle => 'Fade mode';

  @override
  String get demoContainerTransformTypeFade => 'FADE';

  @override
  String get demoContainerTransformTypeFadeThrough => 'FADE THROUGH';

  @override
  String get demoMotionPlaceholderTitle => 'Title';

  @override
  String get demoMotionPlaceholderSubtitle => 'Secondary text';

  @override
  String get demoMotionSmallPlaceholderSubtitle => 'Secondary';

  @override
  String get demoMotionDetailsPageTitle => 'Details page';

  @override
  String get demoMotionListTileTitle => 'List item';

  @override
  String get demoSharedAxisDescription =>
      'The shared axis pattern is used for transitions between the UI elements that have a spatial or navigational relationship. This pattern uses a shared transformation on the x, y or z axis to reinforce the relationship between elements.';

  @override
  String get demoSharedXAxisTitle => 'Shared x-axis';

  @override
  String get demoSharedXAxisBackButtonText => 'BACK';

  @override
  String get demoSharedXAxisNextButtonText => 'NEXT';

  @override
  String get demoSharedXAxisCoursePageTitle => 'Streamline your courses';

  @override
  String get demoSharedXAxisCoursePageSubtitle =>
      'Bundled categories appear as groups in your feed. You can always change this later.';

  @override
  String get demoSharedXAxisArtsAndCraftsCourseTitle => 'Arts and crafts';

  @override
  String get demoSharedXAxisBusinessCourseTitle => 'Business';

  @override
  String get demoSharedXAxisIllustrationCourseTitle => 'Illustration';

  @override
  String get demoSharedXAxisDesignCourseTitle => 'Design';

  @override
  String get demoSharedXAxisCulinaryCourseTitle => 'Culinary';

  @override
  String get demoSharedXAxisBundledCourseSubtitle => 'Bundled';

  @override
  String get demoSharedXAxisIndividualCourseSubtitle => 'Shown individually';

  @override
  String get demoSharedXAxisSignInWelcomeText => 'Hi David Park';

  @override
  String get demoSharedXAxisSignInSubtitleText => 'Sign in with your account';

  @override
  String get demoSharedXAxisSignInTextFieldLabel => 'Email or phone number';

  @override
  String get demoSharedXAxisForgotEmailButtonText => 'FORGOT EMAIL?';

  @override
  String get demoSharedXAxisCreateAccountButtonText => 'CREATE ACCOUNT';

  @override
  String get demoSharedYAxisTitle => 'Shared y-axis';

  @override
  String get demoSharedYAxisAlbumCount => '268 albums';

  @override
  String get demoSharedYAxisAlphabeticalSortTitle => 'A–Z';

  @override
  String get demoSharedYAxisRecentSortTitle => 'Recently played';

  @override
  String get demoSharedYAxisAlbumTileTitle => 'Album';

  @override
  String get demoSharedYAxisAlbumTileSubtitle => 'Artist';

  @override
  String get demoSharedYAxisAlbumTileDurationUnit => 'min';

  @override
  String get demoSharedZAxisTitle => 'Shared z-axis';

  @override
  String get demoSharedZAxisSettingsPageTitle => 'Settings';

  @override
  String get demoSharedZAxisBurgerRecipeTitle => 'Burger';

  @override
  String get demoSharedZAxisBurgerRecipeDescription => 'Burger recipe';

  @override
  String get demoSharedZAxisSandwichRecipeTitle => 'Sandwich';

  @override
  String get demoSharedZAxisSandwichRecipeDescription => 'Sandwich recipe';

  @override
  String get demoSharedZAxisDessertRecipeTitle => 'Dessert';

  @override
  String get demoSharedZAxisDessertRecipeDescription => 'Dessert recipe';

  @override
  String get demoSharedZAxisShrimpPlateRecipeTitle => 'Shrimp';

  @override
  String get demoSharedZAxisShrimpPlateRecipeDescription => 'Shrimp plate recipe';

  @override
  String get demoSharedZAxisCrabPlateRecipeTitle => 'Crab';

  @override
  String get demoSharedZAxisCrabPlateRecipeDescription => 'Crab plate recipe';

  @override
  String get demoSharedZAxisBeefSandwichRecipeTitle => 'Beef sandwich';

  @override
  String get demoSharedZAxisBeefSandwichRecipeDescription => 'Beef sandwich recipe';

  @override
  String get demoSharedZAxisSavedRecipesListTitle => 'Saved recipes';

  @override
  String get demoSharedZAxisProfileSettingLabel => 'Profile';

  @override
  String get demoSharedZAxisNotificationSettingLabel => 'Notifications';

  @override
  String get demoSharedZAxisPrivacySettingLabel => 'Privacy';

  @override
  String get demoSharedZAxisHelpSettingLabel => 'Help';

  @override
  String get demoFadeThroughTitle => 'Fade through';

  @override
  String get demoFadeThroughDescription =>
      'The fade-through pattern is used for transitions between UI elements that do not have a strong relationship to each other.';

  @override
  String get demoFadeThroughAlbumsDestination => 'Albums';

  @override
  String get demoFadeThroughPhotosDestination => 'Photos';

  @override
  String get demoFadeThroughSearchDestination => 'Search';

  @override
  String get demoFadeThroughTextPlaceholder => '123 photos';

  @override
  String get demoFadeScaleTitle => 'Fade';

  @override
  String get demoFadeScaleDescription =>
      'The fade pattern is used for UI elements that enter or exit within the bounds of the screen, such as a dialogue that fades in the centre of the screen.';

  @override
  String get demoFadeScaleShowAlertDialogButton => 'SHOW MODAL';

  @override
  String get demoFadeScaleShowFabButton => 'SHOW FAB';

  @override
  String get demoFadeScaleHideFabButton => 'HIDE FAB';

  @override
  String get demoFadeScaleAlertDialogHeader => 'Alert dialogue';

  @override
  String get demoFadeScaleAlertDialogCancelButton => 'CANCEL';

  @override
  String get demoFadeScaleAlertDialogDiscardButton => 'DISCARD';

  @override
  String get demoColorsTitle => 'Colours';

  @override
  String get demoColorsSubtitle => 'All of the predefined colours';

  @override
  String get demoColorsDescription =>
      "Colour and colour swatch constants which represent Material Design's colour palette.";

  @override
  String get demoTypographyTitle => 'Typography';

  @override
  String get demoTypographySubtitle => 'All of the predefined text styles';

  @override
  String get demoTypographyDescription =>
      'Definitions for the various typographical styles found in Material Design.';

  @override
  String get demo2dTransformationsTitle => '2D transformations';

  @override
  String get demo2dTransformationsSubtitle => 'Pan, zoom, rotate';

  @override
  String get demo2dTransformationsDescription =>
      'Tap to edit tiles, and use gestures to move around the scene. Drag to pan, pinch to zoom, rotate with two fingers. Press the reset button to return to the starting orientation.';

  @override
  String get demo2dTransformationsResetTooltip => 'Reset transformations';

  @override
  String get demo2dTransformationsEditTooltip => 'Edit tile';

  @override
  String get buttonText => 'BUTTON';

  @override
  String get demoBottomSheetTitle => 'Bottom sheet';

  @override
  String get demoBottomSheetSubtitle => 'Persistent and modal bottom sheets';

  @override
  String get demoBottomSheetPersistentTitle => 'Persistent bottom sheet';

  @override
  String get demoBottomSheetPersistentDescription =>
      'A persistent bottom sheet shows information that supplements the primary content of the app. A persistent bottom sheet remains visible even when the user interacts with other parts of the app.';

  @override
  String get demoBottomSheetModalTitle => 'Modal bottom sheet';

  @override
  String get demoBottomSheetModalDescription =>
      'A modal bottom sheet is an alternative to a menu or a dialogue and prevents the user from interacting with the rest of the app.';

  @override
  String get demoBottomSheetAddLabel => 'Add';

  @override
  String get demoBottomSheetButtonText => 'SHOW BOTTOM SHEET';

  @override
  String get demoBottomSheetHeader => 'Header';

  @override
  String demoBottomSheetItem(Object value) {
    return 'Item $value';
  }

  @override
  String get demoListsTitle => 'Lists';

  @override
  String get demoListsSubtitle => 'Scrolling list layouts';

  @override
  String get demoListsDescription =>
      'A single fixed-height row that typically contains some text as well as a leading or trailing icon.';

  @override
  String get demoOneLineListsTitle => 'One line';

  @override
  String get demoTwoLineListsTitle => 'Two lines';

  @override
  String get demoListsSecondary => 'Secondary text';

  @override
  String get demoProgressIndicatorTitle => 'Progress indicators';

  @override
  String get demoProgressIndicatorSubtitle => 'Linear, circular, indeterminate';

  @override
  String get demoCircularProgressIndicatorTitle => 'Circular progress indicator';

  @override
  String get demoCircularProgressIndicatorDescription =>
      'A material design circular progress indicator, which spins to indicate that the application is busy.';

  @override
  String get demoLinearProgressIndicatorTitle => 'Linear progress indicator';

  @override
  String get demoLinearProgressIndicatorDescription =>
      'A material design linear progress indicator, also known as a progress bar.';

  @override
  String get demoPickersTitle => 'Pickers';

  @override
  String get demoPickersSubtitle => 'Date and time selection';

  @override
  String get demoDatePickerTitle => 'Date picker';

  @override
  String get demoDatePickerDescription =>
      'Shows a dialogue containing a material design date picker.';

  @override
  String get demoTimePickerTitle => 'Time picker';

  @override
  String get demoTimePickerDescription =>
      'Shows a dialogue containing a material design time picker.';

  @override
  String get demoDateRangePickerTitle => 'Date range picker';

  @override
  String get demoDateRangePickerDescription =>
      'Shows a dialogue containing a Material Design date range picker.';

  @override
  String get demoPickersShowPicker => 'SHOW PICKER';

  @override
  String get demoTabsTitle => 'Tabs';

  @override
  String get demoTabsScrollingTitle => 'Scrolling';

  @override
  String get demoTabsNonScrollingTitle => 'Non-scrolling';

  @override
  String get demoTabsSubtitle => 'Tabs with independently scrollable views';

  @override
  String get demoTabsDescription =>
      'Tabs organise content across different screens, data sets and other interactions.';

  @override
  String get demoSnackbarsTitle => 'Snackbars';

  @override
  String get demoSnackbarsSubtitle => 'Snackbars show messages at the bottom of the screen';

  @override
  String get demoSnackbarsDescription =>
      "Snackbars inform users of a process that an app has performed or will perform. They appear temporarily, towards the bottom of the screen. They shouldn't interrupt the user experience, and they don't require user input to disappear.";

  @override
  String get demoSnackbarsButtonLabel => 'SHOW A SNACKBAR';

  @override
  String get demoSnackbarsText => 'This is a snackbar.';

  @override
  String get demoSnackbarsActionButtonLabel => 'ACTION';

  @override
  String get demoSnackbarsAction => 'You pressed the snackbar action.';

  @override
  String get demoSelectionControlsTitle => 'Selection controls';

  @override
  String get demoSelectionControlsSubtitle => 'Tick boxes, radio buttons and switches';

  @override
  String get demoSelectionControlsCheckboxTitle => 'Tick box';

  @override
  String get demoSelectionControlsCheckboxDescription =>
      "Tick boxes allow the user to select multiple options from a set. A normal tick box's value is true or false and a tristate tick box's value can also be null.";

  @override
  String get demoSelectionControlsRadioTitle => 'Radio';

  @override
  String get demoSelectionControlsRadioDescription =>
      'Radio buttons allow the user to select one option from a set. Use radio buttons for exclusive selection if you think that the user needs to see all available options side by side.';

  @override
  String get demoSelectionControlsSwitchTitle => 'Switch';

  @override
  String get demoSelectionControlsSwitchDescription =>
      "On/off switches toggle the state of a single settings option. The option that the switch controls, as well as the state it's in, should be made clear from the corresponding inline label.";

  @override
  String get demoBottomTextFieldsTitle => 'Text fields';

  @override
  String get demoTextFieldTitle => 'Text fields';

  @override
  String get demoTextFieldSubtitle => 'Single line of editable text and numbers';

  @override
  String get demoTextFieldDescription =>
      'Text fields allow users to enter text into a UI. They typically appear in forms and dialogues.';

  @override
  String get demoTextFieldShowPasswordLabel => 'Show password';

  @override
  String get demoTextFieldHidePasswordLabel => 'Hide password';

  @override
  String get demoTextFieldFormErrors => 'Please fix the errors in red before submitting.';

  @override
  String get demoTextFieldNameRequired => 'Name is required.';

  @override
  String get demoTextFieldOnlyAlphabeticalChars => 'Please enter only alphabetical characters.';

  @override
  String get demoTextFieldEnterUSPhoneNumber => '(###) ###-#### – Enter a US phone number.';

  @override
  String get demoTextFieldEnterPassword => 'Please enter a password.';

  @override
  String get demoTextFieldPasswordsDoNotMatch => "The passwords don't match";

  @override
  String get demoTextFieldWhatDoPeopleCallYou => 'What do people call you?';

  @override
  String get demoTextFieldNameField => 'Name*';

  @override
  String get demoTextFieldWhereCanWeReachYou => 'Where can we contact you?';

  @override
  String get demoTextFieldPhoneNumber => 'Phone number*';

  @override
  String get demoTextFieldYourEmailAddress => 'Your email address';

  @override
  String get demoTextFieldEmail => 'Email';

  @override
  String get demoTextFieldTellUsAboutYourself =>
      'Tell us about yourself (e.g. write down what you do or what hobbies you have)';

  @override
  String get demoTextFieldKeepItShort => 'Keep it short, this is just a demo.';

  @override
  String get demoTextFieldLifeStory => 'Life story';

  @override
  String get demoTextFieldSalary => 'Salary';

  @override
  String get demoTextFieldUSD => 'USD';

  @override
  String get demoTextFieldNoMoreThan => 'No more than 8 characters.';

  @override
  String get demoTextFieldPassword => 'Password*';

  @override
  String get demoTextFieldRetypePassword => 'Re-type password*';

  @override
  String get demoTextFieldSubmit => 'SUBMIT';

  @override
  String demoTextFieldNameHasPhoneNumber(Object name, Object phoneNumber) {
    return '$name phone number is $phoneNumber';
  }

  @override
  String get demoTextFieldRequiredField => '* indicates required field';

  @override
  String get demoTooltipTitle => 'Tooltips';

  @override
  String get demoTooltipSubtitle => 'Short message displayed on long press or hover';

  @override
  String get demoTooltipDescription =>
      'Tooltips provide text labels that help to explain the function of a button or other user interface action. Tooltips display informative text when users hover over, focus on or long press an element.';

  @override
  String get demoTooltipInstructions => 'Long press or hover to display the tooltip.';

  @override
  String get bottomNavigationCommentsTab => 'Comments';

  @override
  String get bottomNavigationCalendarTab => 'Calendar';

  @override
  String get bottomNavigationAccountTab => 'Account';

  @override
  String get bottomNavigationAlarmTab => 'Alarm';

  @override
  String get bottomNavigationCameraTab => 'Camera';

  @override
  String bottomNavigationContentPlaceholder(Object title) {
    return 'Placeholder for $title tab';
  }

  @override
  String get buttonTextCreate => 'Create';

  @override
  String dialogSelectedOption(Object value) {
    return "You selected: '$value'";
  }

  @override
  String get chipTurnOnLights => 'Turn on lights';

  @override
  String get chipSmall => 'Small';

  @override
  String get chipMedium => 'Medium';

  @override
  String get chipLarge => 'Large';

  @override
  String get chipElevator => 'Lift';

  @override
  String get chipWasher => 'Washing machine';

  @override
  String get chipFireplace => 'Fireplace';

  @override
  String get chipBiking => 'Cycling';

  @override
  String get demo => 'Demo';

  @override
  String get bottomAppBar => 'Bottom app bar';

  @override
  String get loading => 'Loading';

  @override
  String get dialogDiscardTitle => 'Discard draft?';

  @override
  String get dialogLocationTitle => "Use Google's location service?";

  @override
  String get dialogLocationDescription =>
      'Let Google help apps determine location. This means sending anonymous location data to Google, even when no apps are running.';

  @override
  String get dialogCancel => 'CANCEL';

  @override
  String get dialogDiscard => 'DISCARD';

  @override
  String get dialogDisagree => 'DISAGREE';

  @override
  String get dialogAgree => 'AGREE';

  @override
  String get dialogSetBackup => 'Set backup account';

  @override
  String get dialogAddAccount => 'Add account';

  @override
  String get dialogShow => 'SHOW DIALOGUE';

  @override
  String get dialogFullscreenTitle => 'Full-Screen Dialogue';

  @override
  String get dialogFullscreenSave => 'SAVE';

  @override
  String get dialogFullscreenDescription => 'A full-screen dialogue demo';

  @override
  String get cupertinoButton => 'Button';

  @override
  String get cupertinoButtonWithBackground => 'With background';

  @override
  String get cupertinoAlertCancel => 'Cancel';

  @override
  String get cupertinoAlertDiscard => 'Discard';

  @override
  String get cupertinoAlertLocationTitle =>
      "Allow 'Maps' to access your location while you are using the app?";

  @override
  String get cupertinoAlertLocationDescription =>
      'Your current location will be displayed on the map and used for directions, nearby search results and estimated travel times.';

  @override
  String get cupertinoAlertAllow => 'Allow';

  @override
  String get cupertinoAlertDontAllow => "Don't allow";

  @override
  String get cupertinoAlertFavoriteDessert => 'Select Favourite Dessert';

  @override
  String get cupertinoAlertDessertDescription =>
      'Please select your favourite type of dessert from the list below. Your selection will be used to customise the suggested list of eateries in your area.';

  @override
  String get cupertinoAlertCheesecake => 'Cheesecake';

  @override
  String get cupertinoAlertTiramisu => 'Tiramisu';

  @override
  String get cupertinoAlertApplePie => 'Apple Pie';

  @override
  String get cupertinoAlertChocolateBrownie => 'Chocolate brownie';

  @override
  String get cupertinoShowAlert => 'Show alert';

  @override
  String get colorsRed => 'RED';

  @override
  String get colorsPink => 'PINK';

  @override
  String get colorsPurple => 'PURPLE';

  @override
  String get colorsDeepPurple => 'DEEP PURPLE';

  @override
  String get colorsIndigo => 'INDIGO';

  @override
  String get colorsBlue => 'BLUE';

  @override
  String get colorsLightBlue => 'LIGHT BLUE';

  @override
  String get colorsCyan => 'CYAN';

  @override
  String get colorsTeal => 'TEAL';

  @override
  String get colorsGreen => 'GREEN';

  @override
  String get colorsLightGreen => 'LIGHT GREEN';

  @override
  String get colorsLime => 'LIME';

  @override
  String get colorsYellow => 'YELLOW';

  @override
  String get colorsAmber => 'AMBER';

  @override
  String get colorsOrange => 'ORANGE';

  @override
  String get colorsDeepOrange => 'DEEP ORANGE';

  @override
  String get colorsBrown => 'BROWN';

  @override
  String get colorsGrey => 'GREY';

  @override
  String get colorsBlueGrey => 'BLUE GREY';

  @override
  String get placeChennai => 'Chennai';

  @override
  String get placeTanjore => 'Tanjore';

  @override
  String get placeChettinad => 'Chettinad';

  @override
  String get placePondicherry => 'Pondicherry';

  @override
  String get placeFlowerMarket => 'Flower market';

  @override
  String get placeBronzeWorks => 'Bronze works';

  @override
  String get placeMarket => 'Market';

  @override
  String get placeThanjavurTemple => 'Thanjavur Temple';

  @override
  String get placeSaltFarm => 'Salt farm';

  @override
  String get placeScooters => 'Scooters';

  @override
  String get placeSilkMaker => 'Silk maker';

  @override
  String get placeLunchPrep => 'Lunch prep';

  @override
  String get placeBeach => 'Beach';

  @override
  String get placeFisherman => 'Fisherman';

  @override
  String get starterAppTitle => 'Starter app';

  @override
  String get starterAppDescription => 'A responsive starter layout';

  @override
  String get starterAppGenericButton => 'BUTTON';

  @override
  String get starterAppTooltipAdd => 'Add';

  @override
  String get starterAppTooltipFavorite => 'Favourite';

  @override
  String get starterAppTooltipShare => 'Share';

  @override
  String get starterAppTooltipSearch => 'Search';

  @override
  String get starterAppGenericTitle => 'Title';

  @override
  String get starterAppGenericSubtitle => 'Subtitle';

  @override
  String get starterAppGenericHeadline => 'Headline';

  @override
  String get starterAppGenericBody => 'Body';

  @override
  String starterAppDrawerItem(Object value) {
    return 'Item $value';
  }

  @override
  String get shrineMenuCaption => 'MENU';

  @override
  String get shrineCategoryNameAll => 'ALL';

  @override
  String get shrineCategoryNameAccessories => 'ACCESSORIES';

  @override
  String get shrineCategoryNameClothing => 'CLOTHING';

  @override
  String get shrineCategoryNameHome => 'HOME';

  @override
  String get shrineLogoutButtonCaption => 'LOGOUT';

  @override
  String get shrineLoginUsernameLabel => 'Username';

  @override
  String get shrineLoginPasswordLabel => 'Password';

  @override
  String get shrineCancelButtonCaption => 'CANCEL';

  @override
  String get shrineNextButtonCaption => 'NEXT';

  @override
  String get shrineCartPageCaption => 'BASKET';

  @override
  String shrineProductQuantity(Object quantity) {
    return 'Quantity: $quantity';
  }

  @override
  String shrineProductPrice(Object price) {
    return 'x $price';
  }

  @override
  String shrineCartItemCount(num quantity) {
    final String temp0 = intl.Intl.pluralLogic(
      quantity,
      locale: localeName,
      other: '$quantity ITEMS',
      one: '1 ITEM',
      zero: 'NO ITEMS',
    );
    return temp0;
  }

  @override
  String get shrineCartClearButtonCaption => 'CLEAR BASKET';

  @override
  String get shrineCartTotalCaption => 'TOTAL';

  @override
  String get shrineCartSubtotalCaption => 'Subtotal:';

  @override
  String get shrineCartShippingCaption => 'Delivery:';

  @override
  String get shrineCartTaxCaption => 'Tax:';

  @override
  String get shrineProductVagabondSack => 'Vagabond sack';

  @override
  String get shrineProductStellaSunglasses => 'Stella sunglasses';

  @override
  String get shrineProductWhitneyBelt => 'Whitney belt';

  @override
  String get shrineProductGardenStrand => 'Garden strand';

  @override
  String get shrineProductStrutEarrings => 'Strut earrings';

  @override
  String get shrineProductVarsitySocks => 'Varsity socks';

  @override
  String get shrineProductWeaveKeyring => 'Weave keyring';

  @override
  String get shrineProductGatsbyHat => 'Gatsby hat';

  @override
  String get shrineProductShrugBag => 'Shrug bag';

  @override
  String get shrineProductGiltDeskTrio => 'Gilt desk trio';

  @override
  String get shrineProductCopperWireRack => 'Copper wire rack';

  @override
  String get shrineProductSootheCeramicSet => 'Soothe ceramic set';

  @override
  String get shrineProductHurrahsTeaSet => 'Hurrahs tea set';

  @override
  String get shrineProductBlueStoneMug => 'Blue stone mug';

  @override
  String get shrineProductRainwaterTray => 'Rainwater tray';

  @override
  String get shrineProductChambrayNapkins => 'Chambray napkins';

  @override
  String get shrineProductSucculentPlanters => 'Succulent planters';

  @override
  String get shrineProductQuartetTable => 'Quartet table';

  @override
  String get shrineProductKitchenQuattro => 'Kitchen quattro';

  @override
  String get shrineProductClaySweater => 'Clay sweater';

  @override
  String get shrineProductSeaTunic => 'Sea tunic';

  @override
  String get shrineProductPlasterTunic => 'Plaster tunic';

  @override
  String get shrineProductWhitePinstripeShirt => 'White pinstripe shirt';

  @override
  String get shrineProductChambrayShirt => 'Chambray shirt';

  @override
  String get shrineProductSeabreezeSweater => 'Seabreeze sweater';

  @override
  String get shrineProductGentryJacket => 'Gentry jacket';

  @override
  String get shrineProductNavyTrousers => 'Navy trousers';

  @override
  String get shrineProductWalterHenleyWhite => 'Walter henley (white)';

  @override
  String get shrineProductSurfAndPerfShirt => 'Surf and perf shirt';

  @override
  String get shrineProductGingerScarf => 'Ginger scarf';

  @override
  String get shrineProductRamonaCrossover => 'Ramona crossover';

  @override
  String get shrineProductClassicWhiteCollar => 'Classic white collar';

  @override
  String get shrineProductCeriseScallopTee => 'Cerise scallop tee';

  @override
  String get shrineProductShoulderRollsTee => 'Shoulder rolls tee';

  @override
  String get shrineProductGreySlouchTank => 'Grey slouch tank top';

  @override
  String get shrineProductSunshirtDress => 'Sunshirt dress';

  @override
  String get shrineProductFineLinesTee => 'Fine lines tee';

  @override
  String get shrineTooltipSearch => 'Search';

  @override
  String get shrineTooltipSettings => 'Settings';

  @override
  String get shrineTooltipOpenMenu => 'Open menu';

  @override
  String get shrineTooltipCloseMenu => 'Close menu';

  @override
  String get shrineTooltipCloseCart => 'Close basket';

  @override
  String shrineScreenReaderCart(num quantity) {
    final String temp0 = intl.Intl.pluralLogic(
      quantity,
      locale: localeName,
      other: 'Shopping basket, $quantity items',
      one: 'Shopping basket, 1 item',
      zero: 'Shopping basket, no items',
    );
    return temp0;
  }

  @override
  String get shrineScreenReaderProductAddToCart => 'Add to basket';

  @override
  String shrineScreenReaderRemoveProductButton(Object product) {
    return 'Remove $product';
  }

  @override
  String get shrineTooltipRemoveItem => 'Remove item';

  @override
  String get craneFormDiners => 'Diners';

  @override
  String get craneFormDate => 'Select date';

  @override
  String get craneFormTime => 'Select time';

  @override
  String get craneFormLocation => 'Select location';

  @override
  String get craneFormTravelers => 'Travellers';

  @override
  String get craneFormOrigin => 'Choose origin';

  @override
  String get craneFormDestination => 'Choose destination';

  @override
  String get craneFormDates => 'Select dates';

  @override
  String craneHours(num hours) {
    final String temp0 = intl.Intl.pluralLogic(
      hours,
      locale: localeName,
      other: '${hours}h',
      one: '1 h',
    );
    return temp0;
  }

  @override
  String craneMinutes(num minutes) {
    final String temp0 = intl.Intl.pluralLogic(
      minutes,
      locale: localeName,
      other: '${minutes}m',
      one: '1 m',
    );
    return temp0;
  }

  @override
  String craneFlightDuration(Object hoursShortForm, Object minutesShortForm) {
    return '$hoursShortForm $minutesShortForm';
  }

  @override
  String get craneFly => 'FLY';

  @override
  String get craneSleep => 'SLEEP';

  @override
  String get craneEat => 'EAT';

  @override
  String get craneFlySubhead => 'Explore flights by destination';

  @override
  String get craneSleepSubhead => 'Explore properties by destination';

  @override
  String get craneEatSubhead => 'Explore restaurants by destination';

  @override
  String craneFlyStops(num numberOfStops) {
    final String temp0 = intl.Intl.pluralLogic(
      numberOfStops,
      locale: localeName,
      other: '$numberOfStops stops',
      one: '1 stop',
      zero: 'Non-stop',
    );
    return temp0;
  }

  @override
  String craneSleepProperties(num totalProperties) {
    final String temp0 = intl.Intl.pluralLogic(
      totalProperties,
      locale: localeName,
      other: '$totalProperties available properties',
      one: '1 available property',
      zero: 'No available properties',
    );
    return temp0;
  }

  @override
  String craneEatRestaurants(num totalRestaurants) {
    return intl.Intl.pluralLogic(
      totalRestaurants,
      locale: localeName,
      other: '$totalRestaurants restaurants',
      one: '1 restaurant',
      zero: 'No restaurants',
    );
  }

  @override
  String get craneFly0 => 'Aspen, United States';

  @override
  String get craneFly1 => 'Big Sur, United States';

  @override
  String get craneFly2 => 'Khumbu Valley, Nepal';

  @override
  String get craneFly3 => 'Machu Picchu, Peru';

  @override
  String get craneFly4 => 'Malé, Maldives';

  @override
  String get craneFly5 => 'Vitznau, Switzerland';

  @override
  String get craneFly6 => 'Mexico City, Mexico';

  @override
  String get craneFly7 => 'Mount Rushmore, United States';

  @override
  String get craneFly8 => 'Singapore';

  @override
  String get craneFly9 => 'Havana, Cuba';

  @override
  String get craneFly10 => 'Cairo, Egypt';

  @override
  String get craneFly11 => 'Lisbon, Portugal';

  @override
  String get craneFly12 => 'Napa, United States';

  @override
  String get craneFly13 => 'Bali, Indonesia';

  @override
  String get craneSleep0 => 'Malé, Maldives';

  @override
  String get craneSleep1 => 'Aspen, United States';

  @override
  String get craneSleep2 => 'Machu Picchu, Peru';

  @override
  String get craneSleep3 => 'Havana, Cuba';

  @override
  String get craneSleep4 => 'Vitznau, Switzerland';

  @override
  String get craneSleep5 => 'Big Sur, United States';

  @override
  String get craneSleep6 => 'Napa, United States';

  @override
  String get craneSleep7 => 'Porto, Portugal';

  @override
  String get craneSleep8 => 'Tulum, Mexico';

  @override
  String get craneSleep9 => 'Lisbon, Portugal';

  @override
  String get craneSleep10 => 'Cairo, Egypt';

  @override
  String get craneSleep11 => 'Taipei, Taiwan';

  @override
  String get craneEat0 => 'Naples, Italy';

  @override
  String get craneEat1 => 'Dallas, United States';

  @override
  String get craneEat2 => 'Córdoba, Argentina';

  @override
  String get craneEat3 => 'Portland, United States';

  @override
  String get craneEat4 => 'Paris, France';

  @override
  String get craneEat5 => 'Seoul, South Korea';

  @override
  String get craneEat6 => 'Seattle, United States';

  @override
  String get craneEat7 => 'Nashville, United States';

  @override
  String get craneEat8 => 'Atlanta, United States';

  @override
  String get craneEat9 => 'Madrid, Spain';

  @override
  String get craneEat10 => 'Lisbon, Portugal';

  @override
  String get craneFly0SemanticLabel => 'Chalet in a snowy landscape with evergreen trees';

  @override
  String get craneFly1SemanticLabel => 'Tent in a field';

  @override
  String get craneFly2SemanticLabel => 'Prayer flags in front of snowy mountain';

  @override
  String get craneFly3SemanticLabel => 'Machu Picchu citadel';

  @override
  String get craneFly4SemanticLabel => 'Overwater bungalows';

  @override
  String get craneFly5SemanticLabel => 'Lake-side hotel in front of mountains';

  @override
  String get craneFly6SemanticLabel => 'Aerial view of Palacio de Bellas Artes';

  @override
  String get craneFly7SemanticLabel => 'Mount Rushmore';

  @override
  String get craneFly8SemanticLabel => 'Supertree Grove';

  @override
  String get craneFly9SemanticLabel => 'Man leaning on an antique blue car';

  @override
  String get craneFly10SemanticLabel => 'Al-Azhar Mosque towers during sunset';

  @override
  String get craneFly11SemanticLabel => 'Brick lighthouse at sea';

  @override
  String get craneFly12SemanticLabel => 'Pool with palm trees';

  @override
  String get craneFly13SemanticLabel => 'Seaside pool with palm trees';

  @override
  String get craneSleep0SemanticLabel => 'Overwater bungalows';

  @override
  String get craneSleep1SemanticLabel => 'Chalet in a snowy landscape with evergreen trees';

  @override
  String get craneSleep2SemanticLabel => 'Machu Picchu citadel';

  @override
  String get craneSleep3SemanticLabel => 'Man leaning on an antique blue car';

  @override
  String get craneSleep4SemanticLabel => 'Lake-side hotel in front of mountains';

  @override
  String get craneSleep5SemanticLabel => 'Tent in a field';

  @override
  String get craneSleep6SemanticLabel => 'Pool with palm trees';

  @override
  String get craneSleep7SemanticLabel => 'Colourful apartments at Ribeira Square';

  @override
  String get craneSleep8SemanticLabel => 'Mayan ruins on a cliff above a beach';

  @override
  String get craneSleep9SemanticLabel => 'Brick lighthouse at sea';

  @override
  String get craneSleep10SemanticLabel => 'Al-Azhar Mosque towers during sunset';

  @override
  String get craneSleep11SemanticLabel => 'Taipei 101 skyscraper';

  @override
  String get craneEat0SemanticLabel => 'Pizza in a wood-fired oven';

  @override
  String get craneEat1SemanticLabel => 'Empty bar with diner-style stools';

  @override
  String get craneEat2SemanticLabel => 'Burger';

  @override
  String get craneEat3SemanticLabel => 'Korean taco';

  @override
  String get craneEat4SemanticLabel => 'Chocolate dessert';

  @override
  String get craneEat5SemanticLabel => 'Artsy restaurant seating area';

  @override
  String get craneEat6SemanticLabel => 'Shrimp dish';

  @override
  String get craneEat7SemanticLabel => 'Bakery entrance';

  @override
  String get craneEat8SemanticLabel => 'Plate of crawfish';

  @override
  String get craneEat9SemanticLabel => 'Café counter with pastries';

  @override
  String get craneEat10SemanticLabel => 'Woman holding huge pastrami sandwich';

  @override
  String get fortnightlyMenuFrontPage => 'Front page';

  @override
  String get fortnightlyMenuWorld => 'World';

  @override
  String get fortnightlyMenuUS => 'US';

  @override
  String get fortnightlyMenuPolitics => 'Politics';

  @override
  String get fortnightlyMenuBusiness => 'Business';

  @override
  String get fortnightlyMenuTech => 'Tech';

  @override
  String get fortnightlyMenuScience => 'Science';

  @override
  String get fortnightlyMenuSports => 'Sport';

  @override
  String get fortnightlyMenuTravel => 'Travel';

  @override
  String get fortnightlyMenuCulture => 'Culture';

  @override
  String get fortnightlyTrendingTechDesign => 'TechDesign';

  @override
  String get fortnightlyTrendingReform => 'Reform';

  @override
  String get fortnightlyTrendingHealthcareRevolution => 'HealthcareRevolution';

  @override
  String get fortnightlyTrendingGreenArmy => 'GreenArmy';

  @override
  String get fortnightlyTrendingStocks => 'Stocks';

  @override
  String get fortnightlyLatestUpdates => 'Latest updates';

  @override
  String get fortnightlyHeadlineHealthcare => 'The Quiet, yet Powerful Healthcare Revolution';

  @override
  String get fortnightlyHeadlineWar => 'Divided American Lives During War';

  @override
  String get fortnightlyHeadlineGasoline => 'The Future of Petrol';

  @override
  String get fortnightlyHeadlineArmy => 'Reforming The Green Army from Within';

  @override
  String get fortnightlyHeadlineStocks => 'As Stocks Stagnate, many Look to Currency';

  @override
  String get fortnightlyHeadlineFabrics => 'Designers use Tech to make Futuristic Fabrics';

  @override
  String get fortnightlyHeadlineFeminists => 'Feminists take on Partisanship';

  @override
  String get fortnightlyHeadlineBees => 'Farmland Bees in Short Supply';

  @override
  String get replyInboxLabel => 'Inbox';

  @override
  String get replyStarredLabel => 'Starred';

  @override
  String get replySentLabel => 'Sent';

  @override
  String get replyTrashLabel => 'Bin';

  @override
  String get replySpamLabel => 'Spam';

  @override
  String get replyDraftsLabel => 'Drafts';

  @override
  String get demoTwoPaneFoldableLabel => 'Foldable';

  @override
  String get demoTwoPaneFoldableDescription => 'This is how TwoPane behaves on a foldable device.';

  @override
  String get demoTwoPaneSmallScreenLabel => 'Small screen';

  @override
  String get demoTwoPaneSmallScreenDescription =>
      'This is how TwoPane behaves on a small screen device.';

  @override
  String get demoTwoPaneTabletLabel => 'Tablet/Desktop';

  @override
  String get demoTwoPaneTabletDescription =>
      'This is how TwoPane behaves on a larger screen like a tablet or desktop.';

  @override
  String get demoTwoPaneTitle => 'TwoPane';

  @override
  String get demoTwoPaneSubtitle => 'Responsive layouts on foldable, large and small screens';

  @override
  String get splashSelectDemo => 'Select a demo';

  @override
  String get demoTwoPaneList => 'List';

  @override
  String get demoTwoPaneDetails => 'Details';

  @override
  String get demoTwoPaneSelectItem => 'Select an item';

  @override
  String demoTwoPaneItem(Object value) {
    return 'Item $value';
  }

  @override
  String demoTwoPaneItemDetails(Object value) {
    return 'Item $value details';
  }
}
