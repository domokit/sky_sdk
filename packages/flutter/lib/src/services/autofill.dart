// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'text_input.dart';

export 'text_input.dart'
    show TextEditingValue, TextInputClient, TextInputConfiguration, TextInputConnection;

/// A collection of commonly used autofill hint strings on different platforms.
///
/// Each hint is pre-defined on at least one supported platform. See their
/// documentation for their availability on each platform, and the platform
/// values each autofill hint corresponds to.
abstract final class AutofillHints {
  /// The input field expects an address locality (city/town).
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS_LOCALITY](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS_LOCALITY).
  /// * iOS: [addressCity](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * Otherwise, the hint string will be used as-is.
  static const String addressCity = 'addressCity';

  /// The input field expects a city name combined with a state name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [addressCityAndState](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * Otherwise, the hint string will be used as-is.
  static const String addressCityAndState = 'addressCityAndState';

  /// The input field expects a region/state.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS_REGION](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS_REGION).
  /// * iOS: [addressState](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * Otherwise, the hint string will be used as-is.
  static const String addressState = 'addressState';

  /// The input field expects a person's full birth date.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_BIRTH_DATE_FULL](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_BIRTH_DATE_FULL).
  /// * web: ["bday"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String birthday = 'birthday';

  /// The input field expects a person's birth day(of the month).
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_BIRTH_DATE_DAY](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_BIRTH_DATE_DAY).
  /// * web: ["bday-day"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String birthdayDay = 'birthdayDay';

  /// The input field expects a person's birth month.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_BIRTH_DATE_MONTH](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_BIRTH_DATE_MONTH).
  /// * web: ["bday-month"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String birthdayMonth = 'birthdayMonth';

  /// The input field expects a person's birth year.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_BIRTH_DATE_YEAR](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_BIRTH_DATE_YEAR).
  /// * web: ["bday-year"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String birthdayYear = 'birthdayYear';

  /// The input field expects an
  /// [ISO 3166-1-alpha-2](https://www.iso.org/standard/63545.html) country code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["country"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String countryCode = 'countryCode';

  /// The input field expects a country name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS_COUNTRY](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS_COUNTRY).
  /// * iOS: [countryName](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["country-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String countryName = 'countryName';

  /// The input field expects a credit card expiration date.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_CREDIT_CARD_NUMBER](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_CREDIT_CARD_NUMBER).
  /// * web: ["cc-exp"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardExpirationDate = 'creditCardExpirationDate';

  /// The input field expects a credit card expiration day.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_CREDIT_CARD_EXPIRATION_DAY](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_CREDIT_CARD_EXPIRATION_DAY).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardExpirationDay = 'creditCardExpirationDay';

  /// The input field expects a credit card expiration month.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_CREDIT_CARD_EXPIRATION_MONTH](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_CREDIT_CARD_EXPIRATION_MONTH).
  /// * web: ["cc-exp-month"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardExpirationMonth = 'creditCardExpirationMonth';

  /// The input field expects a credit card expiration year.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_CREDIT_CARD_EXPIRATION_YEAR](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_CREDIT_CARD_EXPIRATION_YEAR).
  /// * web: ["cc-exp-year"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardExpirationYear = 'creditCardExpirationYear';

  /// The input field expects the holder's last/family name as given on a credit
  /// card.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["cc-family-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardFamilyName = 'creditCardFamilyName';

  /// The input field expects the holder's first/given name as given on a credit
  /// card.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["cc-given-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardGivenName = 'creditCardGivenName';

  /// The input field expects the holder's middle name as given on a credit
  /// card.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["cc-additional-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardMiddleName = 'creditCardMiddleName';

  /// The input field expects the holder's full name as given on a credit card.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["cc-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardName = 'creditCardName';

  /// The input field expects a credit card number.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_CREDIT_CARD_NUMBER](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_CREDIT_CARD_NUMBER).
  /// * iOS: [creditCardNumber](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["cc-number"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardNumber = 'creditCardNumber';

  /// The input field expects a credit card security code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_CREDIT_CARD_SECURITY_CODE](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_CREDIT_CARD_SECURITY_CODE).
  /// * web: ["cc-csc"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardSecurityCode = 'creditCardSecurityCode';

  /// The input field expects the type of a credit card, for example "Visa".
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["cc-type"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String creditCardType = 'creditCardType';

  /// The input field expects an email address.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_EMAIL_ADDRESS](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_EMAIL_ADDRESS).
  /// * iOS: [emailAddress](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["email"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String email = 'email';

  /// The input field expects a person's last/family name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME_FAMILY](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME_FAMILY).
  /// * iOS: [familyName](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["family-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String familyName = 'familyName';

  /// The input field expects a street address that fully identifies a location.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS_STREET_ADDRESS](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS_STREET_ADDRESS).
  /// * iOS: [fullStreetAddress](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["street-address"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String fullStreetAddress = 'fullStreetAddress';

  /// The input field expects a gender.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_GENDER](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_GENDER).
  /// * web: ["sex"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String gender = 'gender';

  /// The input field expects a person's first/given name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME_GIVEN](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME_GIVEN).
  /// * iOS: [givenName](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["given-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String givenName = 'givenName';

  /// The input field expects a URL representing an instant messaging protocol
  /// endpoint.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["impp"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String impp = 'impp';

  /// The input field expects a job title.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [jobTitle](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["organization-title"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String jobTitle = 'jobTitle';

  /// The input field expects the preferred language of the user.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["language"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String language = 'language';

  /// The input field expects a location, such as a point of interest, an
  /// address,or another way to identify a location.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [location](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * Otherwise, the hint string will be used as-is.
  static const String location = 'location';

  /// The input field expects a person's middle initial.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME_MIDDLE_INITIAL](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME_MIDDLE_INITIAL).
  /// * Otherwise, the hint string will be used as-is.
  static const String middleInitial = 'middleInitial';

  /// The input field expects a person's middle name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME_MIDDLE](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME_MIDDLE).
  /// * iOS: [middleName](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["additional-name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String middleName = 'middleName';

  /// The input field expects a person's full name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME).
  /// * iOS: [name](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["name"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String name = 'name';

  /// The input field expects a person's name prefix or title, such as "Dr.".
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME_PREFIX](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME_PREFIX).
  /// * iOS: [namePrefix](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["honorific-prefix"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String namePrefix = 'namePrefix';

  /// The input field expects a person's name suffix, such as "Jr.".
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PERSON_NAME_SUFFIX](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PERSON_NAME_SUFFIX).
  /// * iOS: [nameSuffix](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["honorific-suffix"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String nameSuffix = 'nameSuffix';

  /// The input field expects a newly created password for save/update.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_NEW_PASSWORD](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_NEW_PASSWORD).
  /// * iOS: [newPassword](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["new-password"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String newPassword = 'newPassword';

  /// The input field expects a newly created username for save/update.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_NEW_USERNAME](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_NEW_USERNAME).
  /// * Otherwise, the hint string will be used as-is.
  static const String newUsername = 'newUsername';

  /// The input field expects a nickname.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [nickname](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["nickname"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String nickname = 'nickname';

  /// The input field expects a SMS one-time code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_SMS_OTP](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_SMS_OTP).
  /// * iOS: [oneTimeCode](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["one-time-code"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String oneTimeCode = 'oneTimeCode';

  /// The input field expects an organization name corresponding to the person,
  /// address, or contact information in the other fields associated with this
  /// field.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [organizationName](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["organization"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String organizationName = 'organizationName';

  /// The input field expects a password.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PASSWORD](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PASSWORD).
  /// * iOS: [password](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["current-password"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String password = 'password';

  /// The input field expects a photograph, icon, or other image corresponding
  /// to the company, person, address, or contact information in the other
  /// fields associated with this field.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["photo"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String photo = 'photo';

  /// The input field expects a postal address.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS).
  /// * Otherwise, the hint string will be used as-is.
  static const String postalAddress = 'postalAddress';

  /// The input field expects an auxiliary address details.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS_EXTENDED_ADDRESS](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS_EXTENDED_ADDRESS).
  /// * Otherwise, the hint string will be used as-is.
  static const String postalAddressExtended = 'postalAddressExtended';

  /// The input field expects an extended ZIP/POSTAL code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_ADDRESS_EXTENDED_POSTAL_CODE](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_ADDRESS_EXTENDED_POSTAL_CODE).
  /// * Otherwise, the hint string will be used as-is.
  static const String postalAddressExtendedPostalCode = 'postalAddressExtendedPostalCode';

  /// The input field expects a postal code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_POSTAL_CODE](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_POSTAL_CODE).
  /// * iOS: [postalCode](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["postal-code"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String postalCode = 'postalCode';

  /// The first administrative level in the address. This is typically the
  /// province in which the address is located. In the United States, this would
  /// be the state. In Switzerland, the canton. In the United Kingdom, the post
  /// town.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["address-level1"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLevel1 = 'streetAddressLevel1';

  /// The second administrative level, in addresses with at least two of them.
  /// In countries with two administrative levels, this would typically be the
  /// city, town, village, or other locality in which the address is located.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["address-level2"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLevel2 = 'streetAddressLevel2';

  /// The third administrative level, in addresses with at least three
  /// administrative levels.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["address-level3"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLevel3 = 'streetAddressLevel3';

  /// The finest-grained administrative level, in addresses which have four
  /// levels.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["address-level4"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLevel4 = 'streetAddressLevel4';

  /// The input field expects the first line of a street address.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [streetAddressLine1](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["address-line1"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLine1 = 'streetAddressLine1';

  /// The input field expects the second line of a street address.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [streetAddressLine2](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  ///   As of iOS 14.2 this hint does not trigger autofill.
  /// * web: ["address-line2"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLine2 = 'streetAddressLine2';

  /// The input field expects the third line of a street address.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["address-line3"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String streetAddressLine3 = 'streetAddressLine3';

  /// The input field expects a sublocality.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [sublocality](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * Otherwise, the hint string will be used as-is.
  static const String sublocality = 'sublocality';

  /// The input field expects a telephone number.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PHONE_NUMBER](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PHONE_NUMBER).
  /// * iOS: [telephoneNumber](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["tel"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumber = 'telephoneNumber';

  /// The input field expects a phone number's area code, with a country
  /// -internal prefix applied if applicable.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["tel-area-code"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberAreaCode = 'telephoneNumberAreaCode';

  /// The input field expects a phone number's country code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PHONE_COUNTRY_CODE](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PHONE_COUNTRY_CODE).
  /// * web: ["tel-country-code"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberCountryCode = 'telephoneNumberCountryCode';

  /// The input field expects the current device's phone number, usually for
  /// Sign Up / OTP flows.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PHONE_NUMBER_DEVICE](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PHONE_NUMBER_DEVICE).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberDevice = 'telephoneNumberDevice';

  /// The input field expects a phone number's internal extension code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["tel-extension"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberExtension = 'telephoneNumberExtension';

  /// The input field expects a phone number without the country code and area
  /// code components.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["tel-local"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberLocal = 'telephoneNumberLocal';

  /// The input field expects the first part of the component of the telephone
  /// number that follows the area code, when that component is split into two
  /// components.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["tel-local-prefix"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberLocalPrefix = 'telephoneNumberLocalPrefix';

  /// The input field expects the second part of the component of the telephone
  /// number that follows the area code, when that component is split into two
  /// components.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["tel-local-suffix"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberLocalSuffix = 'telephoneNumberLocalSuffix';

  /// The input field expects a phone number without country code.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_PHONE_NATIONAL](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_PHONE_NATIONAL).
  /// * web: ["tel-national"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String telephoneNumberNational = 'telephoneNumberNational';

  /// The amount that the user would like for the transaction (e.g. when
  /// entering a bid or sale price).
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["transaction-amount"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String transactionAmount = 'transactionAmount';

  /// The currency that the user would prefer the transaction to use, in [ISO
  /// 4217 currency code](https://www.iso.org/iso-4217-currency-codes.html).
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * web: ["transaction-currency"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String transactionCurrency = 'transactionCurrency';

  /// The input field expects a URL.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * iOS: [URL](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["url"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String url = 'url';

  /// The input field expects a username or an account name.
  ///
  /// This hint will be translated to the below values on different platforms:
  ///
  /// * Android: [AUTOFILL_HINT_USERNAME](https://developer.android.com/reference/androidx/autofill/HintConstants#AUTOFILL_HINT_USERNAME).
  /// * iOS: [username](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  /// * web: ["username"](https://html.spec.whatwg.org/multipage/form-control-infrastructure.html#autofilling-form-controls:-the-autocomplete-attribute).
  /// * Otherwise, the hint string will be used as-is.
  static const String username = 'username';
}

/// A collection of autofill related information that represents an [AutofillClient].
///
/// Typically used in [TextInputConfiguration.autofillConfiguration].
@immutable
class AutofillConfiguration {
  /// Creates autofill related configuration information that can be sent to the
  /// platform.
  const AutofillConfiguration({
    required String uniqueIdentifier,
    required List<String> autofillHints,
    required TextEditingValue currentEditingValue,
    String? hintText,
  }) : this._(
         enabled: true,
         uniqueIdentifier: uniqueIdentifier,
         autofillHints: autofillHints,
         currentEditingValue: currentEditingValue,
         hintText: hintText,
       );

  const AutofillConfiguration._({
    required this.enabled,
    required this.uniqueIdentifier,
    this.autofillHints = const <String>[],
    this.hintText,
    required this.currentEditingValue,
  });

  /// An [AutofillConfiguration] that indicates the [AutofillClient] does not
  /// wish to be autofilled.
  static const AutofillConfiguration disabled = AutofillConfiguration._(
    enabled: false,
    uniqueIdentifier: '',
    currentEditingValue: TextEditingValue.empty,
  );

  /// Whether autofill should be enabled for the [AutofillClient].
  ///
  /// To retrieve a disabled [AutofillConfiguration], use [disabled].
  final bool enabled;

  /// A string that uniquely identifies the current [AutofillClient].
  ///
  /// The identifier needs to be unique within the [AutofillScope] for the
  /// [AutofillClient] to receive the correct autofill value.
  final String uniqueIdentifier;

  /// A list of strings that helps the autofill service identify the type of the
  /// [AutofillClient].
  ///
  /// {@template flutter.services.AutofillConfiguration.autofillHints}
  /// For the best results, hint strings need to be understood by the platform's
  /// autofill service. The common values of hint strings can be found in
  /// [AutofillHints], as well as their availability on different platforms.
  ///
  /// If an autofillable input field needs to use a custom hint that translates to
  /// different strings on different platforms, the easiest way to achieve that
  /// is to return different hint strings based on the value of
  /// [defaultTargetPlatform].
  ///
  /// Each hint in the list, if not ignored, will be translated to the platform's
  /// autofill hint type understood by its autofill services:
  ///
  /// * On iOS, only the first hint in the list is accounted for. The hint will
  ///   be translated to a
  ///   [UITextContentType](https://developer.apple.com/documentation/uikit/uitextcontenttype).
  ///
  /// * On Android, all hints in the list are translated to Android hint strings.
  ///
  /// * On web, only the first hint is accounted for and will be translated to
  ///   an "autocomplete" string.
  ///
  /// Providing an autofill hint that is predefined on the platform does not
  /// automatically grant the input field eligibility for autofill. Ultimately,
  /// it comes down to the autofill service currently in charge to determine
  /// whether an input field is suitable for autofill and what the autofill
  /// candidates are.
  ///
  /// See also:
  ///
  /// * [AutofillHints], a list of autofill hint strings that is predefined on at
  ///   least one platform.
  ///
  /// * [UITextContentType](https://developer.apple.com/documentation/uikit/uitextcontenttype),
  ///   the iOS equivalent.
  ///
  /// * Android [autofillHints](https://developer.android.com/reference/android/view/View#setAutofillHints(java.lang.String...)),
  ///   the Android equivalent.
  ///
  /// * The [autocomplete](https://developer.mozilla.org/en-US/docs/Web/HTML/Attributes/autocomplete) attribute,
  ///   the web equivalent.
  /// {@endtemplate}
  final List<String> autofillHints;

  /// The current [TextEditingValue] of the [AutofillClient].
  final TextEditingValue currentEditingValue;

  /// The optional hint text placed on the view that typically suggests what
  /// sort of input the field accepts, for example "enter your password here".
  ///
  /// If the developer does not specify any [autofillHints], the [hintText] can
  /// be a useful indication to the platform autofill service.
  final String? hintText;

  /// Returns a representation of this object as a JSON object.
  Map<String, dynamic>? toJson() {
    return enabled
        ? <String, dynamic>{
          'uniqueIdentifier': uniqueIdentifier,
          'hints': autofillHints,
          'editingValue': currentEditingValue.toJSON(),
          if (hintText != null) 'hintText': hintText,
        }
        : null;
  }
}

/// An object that represents an autofillable input field in the autofill workflow.
///
/// An [AutofillClient] provides autofill-related information of the input field
/// it represents to the platform, and consumes autofill inputs from the platform.
abstract class AutofillClient {
  /// The unique identifier of this [AutofillClient].
  ///
  /// The identifier must not be changed.
  String get autofillId;

  /// The [TextInputConfiguration] that describes this [AutofillClient].
  ///
  /// In order to participate in autofill, its
  /// [TextInputConfiguration.autofillConfiguration] must not be null.
  TextInputConfiguration get textInputConfiguration;

  /// Requests this [AutofillClient] update its [TextEditingValue] to the given
  /// value.
  void autofill(TextEditingValue newEditingValue);
}

/// An ordered group within which [AutofillClient]s are logically connected.
///
/// {@template flutter.services.AutofillScope}
/// [AutofillClient]s within the same [AutofillScope] are isolated from other
/// input fields during autofill. That is, when an autofillable [TextInputClient]
/// gains focus, only the [AutofillClient]s within the same [AutofillScope] will
/// be visible to the autofill service, in the same order as they appear in
/// [AutofillScope.autofillClients].
///
/// [AutofillScope] also allows [TextInput] to redirect autofill values from the
/// platform to the [AutofillClient] with the given identifier, by calling
/// [AutofillScope.getAutofillClient].
///
/// An [AutofillClient] that's not tied to any [AutofillScope] will only
/// participate in autofill if the autofill is directly triggered by its own
/// [TextInputClient].
/// {@endtemplate}
abstract class AutofillScope {
  /// Gets the [AutofillScope] associated with the given [autofillId], in
  /// this [AutofillScope].
  ///
  /// Returns null if there's no matching [AutofillClient].
  AutofillClient? getAutofillClient(String autofillId);

  /// The collection of [AutofillClient]s currently tied to this [AutofillScope].
  ///
  /// Every [AutofillClient] in this list must have autofill enabled (i.e. its
  /// [AutofillClient.textInputConfiguration] must have a non-null
  /// [AutofillConfiguration].)
  Iterable<AutofillClient> get autofillClients;

  /// Allows a [TextInputClient] to attach to this scope. This method should be
  /// called in lieu of [TextInput.attach], when the [TextInputClient] wishes to
  /// participate in autofill.
  TextInputConnection attach(TextInputClient trigger, TextInputConfiguration configuration);
}

@immutable
class _AutofillScopeTextInputConfiguration extends TextInputConfiguration {
  _AutofillScopeTextInputConfiguration({
    required this.allConfigurations,
    required TextInputConfiguration currentClientConfiguration,
  }) : super(
         viewId: currentClientConfiguration.viewId,
         inputType: currentClientConfiguration.inputType,
         obscureText: currentClientConfiguration.obscureText,
         autocorrect: currentClientConfiguration.autocorrect,
         smartDashesType: currentClientConfiguration.smartDashesType,
         smartQuotesType: currentClientConfiguration.smartQuotesType,
         enableSuggestions: currentClientConfiguration.enableSuggestions,
         inputAction: currentClientConfiguration.inputAction,
         textCapitalization: currentClientConfiguration.textCapitalization,
         keyboardAppearance: currentClientConfiguration.keyboardAppearance,
         actionLabel: currentClientConfiguration.actionLabel,
         autofillConfiguration: currentClientConfiguration.autofillConfiguration,
       );

  final Iterable<TextInputConfiguration> allConfigurations;

  @override
  Map<String, dynamic> toJson() {
    final Map<String, dynamic> result = super.toJson();
    result['fields'] = allConfigurations
        .map((TextInputConfiguration configuration) => configuration.toJson())
        .toList(growable: false);
    return result;
  }
}

/// A partial implementation of [AutofillScope].
///
/// The mixin provides a default implementation for [AutofillScope.attach].
mixin AutofillScopeMixin implements AutofillScope {
  @override
  TextInputConnection attach(TextInputClient trigger, TextInputConfiguration configuration) {
    assert(
      !autofillClients.any(
        (AutofillClient client) => !client.textInputConfiguration.autofillConfiguration.enabled,
      ),
      'Every client in AutofillScope.autofillClients must enable autofill',
    );

    final TextInputConfiguration inputConfiguration = _AutofillScopeTextInputConfiguration(
      allConfigurations: autofillClients.map(
        (AutofillClient client) => client.textInputConfiguration,
      ),
      currentClientConfiguration: configuration,
    );
    return TextInput.attach(trigger, inputConfiguration);
  }
}
